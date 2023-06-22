using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;

namespace RF_HEURISTICS
{

    public class Solution
    {
        //Solution parameters common
        public int[] x_m;
        public int[] x_t;
        public double[,,,] Y_bar;
        public double[,,] Z_bar;
        public int[,] IL_bar_il;
        public int[,] PI_il;
        public double[,] b;
        public double[,] s;
        public double[,,] q_ilt;
        public double[,] q_it;
        public double setupcost;
        public double inventorycost;
        public double backlogcost;
        public double productioncost;
        public double totalcost;
        public int ts, te, w, lambda;
        public double[] frequency_t;
        public double[] recency_t;
        public double[] weight_t;
        public double total_weight_t;
        public double obj;
        public List<int>[] tabu_m;
        public double mu1;
        public double mu2;
        public double[] theta_t;

        //solution parameters for COLGEN
        public int[,] t_plus { get; set; }
        public int[,] t_minus { get; set; }
        public double[,] wf { get; set; }
        public double[,] c2 { get; set; }
        public double[,,] c5 { get; set; }
        public double[,,] c9 { get; set; }
        public double[,] c6 { get; set; }
        public double[,] c7 { get; set; }
        public int[,] alfamap { get; set; }
        public Dictionary<int, double> alfa { get; set; }
        public Dictionary<int, double> beta { get; set; }
        public int[,] Init { get; set; }
        public string Method { get; set; }
        public double SolutionTime { get; set; }
        public double MIPGap { get; set; }
        public double[,] stco { get; set; }
        public Solution(ProblemData data)
        {
            mu1 = 0.2;
            mu2 = 0.5;
            theta_t = new double[data.Tmax + 1];
            total_weight_t = 0;
            Y_bar = new double[data.Imax + 1, data.Imax + 1, data.Imax + 1, data.Tmax + 1];
            Z_bar = new double[data.Imax + 1, data.Lmax + 1, data.Tmax + 1];
            IL_bar_il = new int[data.Imax + 1, data.Lmax + 1];
            PI_il = new int[data.Imax + 1, data.Lmax + 1];
            x_m = new int[data.Lmax + 1];
            x_t = new int[data.Tmax + 1];
            b = new double[data.Imax + 1, data.Tmax + 1];
            tabu_m = new List<int>[data.Imax + 1];
            stco = new double[data.Lmax + 1, data.Tmax + 1];
            c2 = new double[data.Imax + 1, data.Tmax + 1];
            c5 = new double[data.Imax + 1, data.Lmax + 1, data.Tmax + 1];
            c6 = new double[data.Lmax + 1, data.Tmax + 1];
            c7 = new double[data.Lmax + 1, data.Tmax + 1];
            c9 = new double[data.Imax + 1, data.Lmax + 1, data.Tmax + 1];
            for (int i = 1; i <= data.Imax; i++)
            {
                tabu_m[i] = new List<int>();
            }

            //COLGEN related initializations:
            q_ilt = new double[data.Imax + 1, data.Lmax + 1, data.Tmax + 1];
            s = new double[data.Imax + 1, data.Tmax + 1];
            b = new double[data.Imax + 1, data.Tmax + 1];
            t_plus = new int[data.Lmax + 1, data.Tmax + 1];
            t_minus = new int[data.Lmax + 1, data.Tmax + 1];
            wf = new double[data.Lmax + 1, data.Tmax + 1];
            alfamap = new int[data.Lmax + 1, data.Tmax + 1];


            constructIL(data);
            restorex_t(data);
            restorex_m(data);

            frequency_t = new double[data.Tmax + 1];
            recency_t = new double[data.Tmax + 1];
            weight_t = new double[data.Tmax + 1];
            for(int t = 1; t <= data.Tmax; t++)
            {
                frequency_t[t] = 0.0000001;
                recency_t[t] = 0.0000001;
                weight_t[t] = mu1 * frequency_t[t] + mu2 * recency_t[t];
                total_weight_t += weight_t[t];
            }
            for(int t = 1; t <= data.Tmax; t++)
            {
                theta_t[t] = theta_t[t - 1] + weight_t[t] / total_weight_t;
            }
        }

        public void constructIL(ProblemData data)
        {
            IL_bar_il = new int[data.Imax + 1, data.Lmax + 1];
            for (int i = 1; i <= data.Imax; i++)
            {
                List<int> selectedmachines = new List<int>();

                //Step 2
                if (data.A_j[i] <= data.p_j[i]) //Compare the number of available machines Aj and the minimum required number of machines qj for item j. If Aj qj, select all the available machines and go to Step 4; 
                {
                    for (int l = 1; l <= data.Lmax; l++)
                    {
                        if (data.IL_il[i, l] == 1) selectedmachines.Add(l);
                    }

                    goto step4;
                }
                else //otherwise, go to Step 3.
                {
                    //Step 3 

                    if (data.sigma_j[i].Count() < data.p_j[i])
                    {
                        foreach (var a in data.sigma_j[i])
                        {
                            selectedmachines.Add(a);
                        }

                        //Select rest machines using lfm
                        List<int> machinepoolforlfm = new List<int>();
                        for (int l = 1; l <= data.Lmax; l++)
                        {
                            if (data.IL_il[i, l] == 1 && !selectedmachines.Contains(l))
                            {
                                machinepoolforlfm.Add(l);
                            }
                        }

                        foreach (var a in LFM(data.p_j[i] - data.sigma_j[i].Count(), i,data))
                        {
                            selectedmachines.Add(a);
                        }
                    }
                    else
                    {
                        //select from preferential machines using LFM
                        foreach (var a in LFM(data.p_j[i], i,data))
                        {
                            selectedmachines.Add(a);
                        }
                    }
                }


            step4:;
                foreach (var a in selectedmachines)
                {
                    IL_bar_il[i, a] = 1;
                    PI_il[i, a] = 1;
                    if(!tabu_m[i].Contains(a)) tabu_m[i].Add(a);
                }

            }

            for(int l = 1; l <= data.Lmax; l++)
            {

                int counter = 0;
                for(int i = 1; i <= data.Imax; i++)
                {
                    if (IL_bar_il[i, l] == 1) counter++;
                }

                if (counter == 0)
                {
                    for(int i = 1; i <= data.Imax; i++)
                    {
                        if (data.IL_il[i, l] == 1)
                        {
                            PI_il[i, l] = 1;
                            IL_bar_il[i, l] = 1;
                            tabu_m[i].Add(l);
                        }

                    }
                }
            }

        }

        public void restorex_t(ProblemData data)
        {
            for (int t = 1; t <= data.Tmax; t++)
            {
                x_t[t] = 1;
            }
        }

        public void restorex_m(ProblemData data)
        {
            for (int l = 1; l <= data.Lmax; l++)
            {
                x_m[l] = 1;
            }
        }

        public void updatePI(ProblemData data)
        {
            PI_il = new int[data.Imax + 1, data.Lmax+1];
            for(int i = 1; i <= data.Imax; i++)
            {
                for(int l = 1; l <= data.Lmax; l++)
                {
                    PI_il[i, l] = IL_bar_il[i, l];
                }
            }
        }

        public void updateILbar(ProblemData data,Solution solution)
        {

            Dictionary<int, double> _b = new Dictionary<int, double>();
            //get backlogs
            for(int i = 1; i <= data.Imax; i++)
            {
                _b.Add(i, 0);
                for(int t = 1; t <= data.Tmax; t++)
                {
                    _b[i]+= solution.b[i, t];
                }
            }

            //sortbacklogs and add new machine

            int machine = 0;
            foreach (var a in _b.OrderByDescending(x => x.Value).Select(x => x.Key).ToList())
            {

                if (tabu_m[a].Count < data.A_j[a])
                {
                    machine = addMachine(a, data);
                    Console.WriteLine("________________________________________________________part id " + a + " machine id " + machine);

                    if(machine!=0) goto label;
                }

            }

        label:;

            
            x_m = new int[data.Lmax + 1];
            x_m[machine] = 1;

            updatePI(data);

        }

        public void restoreILbar(ProblemData data)
        {
            for (int i = 1; i <= data.Imax; i++)
            {
                for (int l = 1; l <= data.Lmax; l++)
                {
                    IL_bar_il[i, l] = PI_il[i, l];
                }
            }

            x_m = new int[data.Lmax + 1];

            for(int l = 1; l <= data.Lmax; l++)
            {
                for(int i = 1; i <= data.Imax; i++)
                {
                    if (IL_bar_il[i, l] == 1)
                    {
                        x_m[l] = 1;
                    }
                }
            }
        }

        private int addMachine(int i, ProblemData data)
        {
            int machine = 0;
            List<int> selectedmachines = new List<int>();
            for(int l = 1; l <= data.Lmax; l++)
            {
                if (IL_bar_il[i, l] == 1) selectedmachines.Add(l);
            }
            
            if (selectedmachines.Count < data.A_j[i])
            {
               foreach(var a in LFM(1, i, data))
                {
                    IL_bar_il[i, a] = 1;
                    machine = a;
                }
            }

            for(int part = 1; part <= data.Imax; part++)
            {
                IL_bar_il[part, machine] = data.IL_il[part, machine];
            }

            if(!tabu_m[i].Contains(machine)) tabu_m[i].Add(machine);
            updatePI(data);
            return machine;
        }

        private List<int> LFM(int count, int i, ProblemData data)
        {
            List<int> selectedm = new List<int>();

            Random rnd = new Random();

            do
            {
                double randval = rnd.NextDouble();
                double flag = 0;
                for(int l = 1; l <= data.Lmax; l++)
                {
                    if (data.IL_il[i,l]==1)
                    {
                        if ((randval - flag) * (data.Theta_il[i, l] - randval) > 0)
                        {
                            if (!tabu_m[i].Contains(l))
                            {
                                selectedm.Add(l);
                                tabu_m[i].Add(l);
                            }
                        }
                        flag = data.Theta_il[i, l];
                    }
                }
            } while (selectedm.Count < count);

            return selectedm;
        }

        public void updatePhase1(ProblemData data,Solution solution)
        {
            restorex_t(data);
            restoreILbar(data);
            updateILbar(data, solution);

            for(int i = 1; i <= data.Imax; i++)
            {
                for(int j = 1; j <= data.Imax; j++)
                {
                    for (int l = 1; l <= data.Lmax; l++)
                    {
                        for(int t = 1; t <= data.Tmax; t++)
                        {
                            if (solution.Y_bar[i, j, l, t] == 1 || solution.Y_bar[j,i,l,t]==1)
                            {
                                IL_bar_il[i, l] = 1;
                                IL_bar_il[j, l] = 1;
                                if (!tabu_m[i].Contains(l)) tabu_m[i].Add(l);
                                if (!tabu_m[j].Contains(l)) tabu_m[j].Add(l);
                                //x_m[l] = 1;
                            }
                        }
                    }
                }
            }

            updatePI(data);

        }

        public void updatePhase2(ProblemData data,Solution solution,int counter)
        {

            x_m = new int[data.Lmax + 1];
            for(int i = 1; i <= data.Imax; i++)
            {
                for(int l = 1; l <= data.Lmax; l++)
                {
                    IL_bar_il[i, l] = data.IL_il[i, l];

                    if (IL_bar_il[i, l] == 1)
                    {
                        x_m[l] = 1;
                    }
                }
            }
            update_t(data, counter);



        }

        public void update_t(ProblemData data,int counter)
        {
            Random rnd = new Random();
            double random = rnd.NextDouble();

            int tstar = 0;

            updateThetat(data);

            for (int t = 1; t <= data.Tmax; t++)
            {
                if((theta_t[t]-random)*(random - theta_t[t-1])>0)
                {
                    tstar = t;
                    goto label;
                }
            }


        label:;

            ts = tstar;
            te = tstar + w;

            frequency_t[tstar] += 1;
            recency_t[tstar] = counter;
            if (te > data.Tmax) te = data.Tmax;
            x_t = new int[data.Tmax + 1];
            for (int t = ts; t <= te; t++)
            {
                x_t[t] = 1; 
            }

        }

        public void updateThetat(ProblemData data)
        {

            total_weight_t = 0;
            for(int t = 1; t <= data.Tmax; t++)
            {
                weight_t[t] = (mu1 / frequency_t[t]) + (mu2 / recency_t[t]);
                total_weight_t += weight_t[t];
            }
            theta_t = new double[data.Tmax + 1];
            for(int t = 1; t <= data.Tmax; t++)
            {
                theta_t[t] = theta_t[t - 1] + weight_t[t] / total_weight_t;
            }

        }

        public void writedata(ProblemData data,string dataset,int searched_column)
        {       
            using (TextWriter tw = new StreamWriter("/Location of Solutions file/Solutions"+ ".txt",true))
            {
                tw.WriteLine(dataset + " " + Method + " " + SolutionTime + " " + totalcost + " " + backlogcost + " " +  inventorycost + " " + productioncost + " " + setupcost + " " + MIPGap + " " + searched_column);

            }

        }

    }
    
}
