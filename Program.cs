using System;
using System.Collections.Generic;
using System.IO;
using RF_HEURISTICS.COLGEN;
namespace RF_HEURISTICS
{
    class Program
    {
        public static int TL; // Time Limit (seconds)
        public static int IL; // Iteration Limit
        public static int IT; // Iteration Time (seconds)
        static void Main(string[] args)
        {
            string[] fileArray = Directory.GetFiles(@"/Location of DATASETS file/DATASETS/", "*.txt");


            for (int i = 0; i < fileArray.Length; i++)
            {
                if (fileArray[i].Contains("data_30"))
                {
                    TL = 3600;
                    IL = 10;
                    IT = 300;
                    Console.WriteLine(fileArray[i].ToString() + " " + TL);
                    ProblemData data = new ProblemData(fileArray[i]);
                    Solution solution = new Solution(data);
                    Exact(data, solution);
                    solution = new Solution(data);
                    RollingHorizon(data, solution);
                    solution = new Solution(data);
                    Colgen(data, solution);
                    
                }
                else if (fileArray[i].Contains("data_30"))
                {
                    TL = 1800;
                    IL = 10;
                    IT = 300;
                    Console.WriteLine(fileArray[i].ToString() + " " + TL);
                    ProblemData data = new ProblemData(fileArray[i]);
                    Solution solution = new Solution(data);
                    Exact(data, solution);
                    solution = new Solution(data);
                    RollingHorizon(data, solution);
                    solution = new Solution(data);
                    Colgen(data, solution);

                }



            }



            Console.Read();

        }

        public static void RollingHorizon(ProblemData data,Solution solution)
        {
            DateTime start = DateTime.Now;
            solution.ts = 1;
            solution.w = 4;
            solution.lambda = 0;
            solution.te = solution.ts + solution.w;
            MIPmodel_RH mip = new MIPmodel_RH(data, solution);
            mip.modeltype = "RH";
            
            Console.WriteLine("--------------------------------------------Rolling Horizon Phase--------------------------------------------");
            while (solution.te < data.Tmax)
            {
                Console.WriteLine($"--------------------------------------------Rolling Horizon Phase Tstart {solution.ts} Tend {solution.te}--------------------------------------------");

                mip.updatemodel(solution, data);
                mip.solve(solution,data,IT);
                solution.ts = solution.te-solution.lambda;
                solution.te = solution.te + solution.w - solution.lambda;
                Console.WriteLine(solution.ts + " " + solution.te);
            }

            if (solution.te >= data.Tmax)
            {
                solution.te = data.Tmax;
            }

            mip.updatemodel(solution, data);
            mip.solve(solution, data, IT);

            mip.modeltype = "HDFO";

            double zmin = solution.obj;
            Console.WriteLine($"--------------------------------------------HDFO Phase--------------------------------------------");

            while ((DateTime.Now - start).TotalSeconds <= TL)
            {
                Console.WriteLine($"--------------------------------------------HDFO Phase 1--------------------------------------------");

                int lambda = 0;
                while (lambda < IL && (DateTime.Now - start).TotalSeconds <= TL)
                {
                    solution.updatePhase1(data, solution);

                    mip.updatemodel(solution, data);


                    double startingtime = (DateTime.Now - start).TotalSeconds;
                    if (TL - startingtime < IT)
                    {
                        mip.solve(solution, data, Math.Max(1, int.Parse(Math.Round(TL - startingtime, 0).ToString())));

                    }
                    else
                    {
                        mip.solve(solution, data, IT);

                    }

                    if (solution.obj < zmin)
                    {
                        Console.WriteLine($"--------------------------------------------HDFO Phase 1 zmin improved {solution.obj}--------------------------------------------");

                        zmin = solution.obj;
                        lambda = 0;
                    }
                    else
                    {
                        Console.WriteLine($"--------------------------------------------HDFO Phase 1 zmin not improved lambda {lambda + 1}--------------------------------------------");

                        lambda++;
                    }
                    bool flag = false;
                    for (int i = 1; i <= data.Imax; i++)
                    {
                        if (data.A_j[i] > solution.tabu_m[i].Count) flag = true;
                    }
                    if (flag == false)
                    {
                        Console.WriteLine("All machines are searched solution finalized");
                        goto label;
                    }

                }
                Console.WriteLine($"--------------------------------------------HDFO Phase 2--------------------------------------------");

                lambda = 0;
                int counter = 1;
                while (lambda < IL && (DateTime.Now - start).TotalSeconds <= TL)
                {
                    solution.updatePhase2(data, solution, counter);
                    mip.updatemodel(solution, data);
                    double startingtime = (DateTime.Now - start).TotalSeconds;
                    if (TL - startingtime < IT)
                    {
                        mip.solve(solution, data, Math.Max(1,int.Parse(Math.Round(TL - startingtime, 0).ToString())));


                    }
                    else
                    {
                        mip.solve(solution, data, IT);

                    }
                    if (solution.obj < zmin)
                    {
                        Console.WriteLine($"--------------------------------------------HDFO Phase 2 zmin improved {solution.obj}--------------------------------------------");

                        zmin = solution.obj;
                        lambda = 0;
                    }
                    else
                    {
                        Console.WriteLine($"--------------------------------------------HDFO Phase 2 zmin not improved lambda {lambda + 1}--------------------------------------------");

                        lambda++;
                    }
                    counter++;
                }
            }

        label:;
            DateTime end = DateTime.Now;
            double st = (end - start).TotalSeconds;
            solution.SolutionTime = st;
            solution.Method = "HDFO";
            solution.writedata(data, data.filename, 0);
        }

        public static void Colgen(ProblemData data, Solution solution)
        {
            DateTime start = DateTime.Now;

            ColumnPool colpool = new ColumnPool(data, solution);
            MasterMIP mastermip = new MasterMIP();
            double tis = 0;
            do
            {
                mastermip.isRelaxed = false;

                double startingtime = (DateTime.Now - start).TotalSeconds;
                if (TL - startingtime < IT)
                {
                    mastermip.solvemodel(colpool, data, solution, int.Parse(Math.Round(TL - startingtime, 0).ToString()));

                }
                else
                {
                    mastermip.solvemodel(colpool, data, solution,IT );
                }


                mastermip.isRelaxed = true;
                mastermip.solvemodel(colpool, data, solution,IT);
                colpool.GenerateColumns(solution, data);

                DateTime now = DateTime.Now;
                tis = (now - start).TotalSeconds;
            } while (tis <= TL);

            DateTime end = DateTime.Now;
            double st = (end - start).TotalSeconds;
            solution.SolutionTime = st;
            solution.Method = "Colgen";
            int searched_column = colpool.lst_searched_column.Count;
            solution.writedata(data, data.filename,searched_column);
        }

        public static void Exact(ProblemData data,Solution solution)
        {
            DateTime start = DateTime.Now;
            MIPmodel_RH mip = new MIPmodel_RH(data, solution);

            mip.modeltype = "Exact";

            mip.updatemodel(solution, data);
            mip.solve(solution, data, TL);

            DateTime end = DateTime.Now;
            double st = (end - start).TotalSeconds;
            solution.SolutionTime = st;
            solution.Method = "Exact";

            solution.writedata(data, data.filename, 0);
        }
    }
}
