using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using CsvHelper;
using System.Globalization;
using System.Diagnostics;

namespace RF_HEURISTICS.COLGEN
{
    public class ColumnPool
    {
        public Dictionary<int, Column> alfapool = new Dictionary<int, Column>();
        public Dictionary<int, Beta> betapool = new Dictionary<int, Beta>();
        public List<Column> lstalfa = new List<Column>();
        public Dictionary<int, double> predictions = new Dictionary<int, double>();
        public Dictionary<int, string> prediction_strings = new Dictionary<int, string>();
        public Dictionary<string,int> lst_searched_column = new Dictionary<string, int>();

        public ColumnPool(ProblemData data,Solution solution)
        {
            initAlfaPool(data,solution);
        }

        private bool AddColumn(List<int> sequence, int l, int t, int start, ProblemData data)
        {
            Pattern pat = new Pattern(sequence, data);
            Column col = new Column(pat, l, t);

            List<int> compl = new List<int>();
            compl = data.GetCompatibleMachines(pat.Sequence);
            if (compl.Contains(l))
            {
                if (alfapool.Where(x => x.Value.id == col.id).Count() == 0)
                {
                    col.start = start;
                    alfapool.Add(alfapool.Count + 1, col);
                    return true;
                }
            }
            return false;
        }

        private void initAlfaPool(ProblemData data,Solution solution)
        {
            for (int i = 1; i <= data.Imax; i++)
            {
                for (int l = 1; l <= data.Lmax; l++)
                {
                    if (data.IL_il[i, l] == 1)
                    {
                        List<int> sequence = new List<int> { i };
                        for (int t = 1; t <= data.Tmax; t++)
                        {
                            AddColumn(sequence, l, t, 0, data);
                        }
                    }
                }
            }
            initBetaPool(data);
        }

        private void initBetaPool(ProblemData data)
        {
            betapool.Clear();
            foreach (var a in alfapool)
            {
                int machine = a.Value.machine;
                int period = a.Value.period;
                var nextcolumns = alfapool.Where(x => x.Value.machine == machine && x.Value.period == period + 1);
                foreach (var b in nextcolumns)
                {
                    Beta beta = new Beta(a.Value, b.Value, data);
                    if (a.Value.start == 1 && b.Value.start == 1)
                    {
                        beta.start = 1;
                    }
                    else
                    {
                        beta.start = 0;
                    }
                    betapool.Add(betapool.Count + 1, beta);

                }
            }
        }

        public void GenerateColumns(Solution solution, ProblemData data)
        {
            LastSolution(solution, data);
            CarryOver(solution, data);
            RandomParts(solution, data);
            BackloggedParts(solution, data);
            BackloggedPartsInjection(solution, data);
            BestPosition(solution, data);
            initBetaPool(data);
            Console.WriteLine("Total column pool : " + alfapool.Count);
        }

        private void LastSolution(Solution solution, ProblemData data)
        {
            lstalfa.Clear();
            int alfacount = 0;
            foreach (var a in solution.alfa)
            {
                if (a.Value > 0.5)
                {
                    lstalfa.Add(alfapool[a.Key]);
                    alfacount++;
                }
            }
            Console.WriteLine(alfacount + " columns selected");
            alfapool.Clear();
            foreach (var a in lstalfa)
            {
                int startperiod = Math.Max(a.period - 5, 1);
                int finishperiod = Math.Min(a.period + 5, data.Tmax);

                for (int t = startperiod; t <= finishperiod; t++)
                {
                    if (a.period == t)
                    {
                        AddColumn(a.pattern.Sequence, a.machine, t, 0, data);

                    }
                    else
                    {
                        AddColumn(a.pattern.Sequence, a.machine, t, 0, data);
                    }

                }
            }
            solution.alfamap = new int[data.Lmax + 1, data.Tmax + 1];
            foreach (var a in lstalfa)
            {
                int key = alfapool.Where(x => x.Value.id == a.id).FirstOrDefault().Key;
                alfapool[key].start = 1;
                solution.alfamap[alfapool[key].machine, alfapool[key].period] = key;
            }
        }

        private void CarryOver(Solution solution, ProblemData data)
        {
            var lstbeta = solution.beta.Where(x => x.Value > 0.1).Select(x => x.Key).ToList();
            foreach (var b in lstbeta)
            {

                List<int> sequence;
                if (!(betapool[b].col1.pattern.Sequence.Contains(betapool[b].col2.pattern.FirstPart)))
                {
                    int startperiod = Math.Max(betapool[b].col1.period - 5, 1);
                    int finishperiod = Math.Min(betapool[b].col1.period + 5, data.Tmax);
                    int l = betapool[b].col1.machine;
                    sequence = new List<int>(betapool[b].col1.pattern.Sequence);
                    sequence.Insert(sequence.Count, betapool[b].col2.pattern.FirstPart);
                    for (int t = startperiod; t <= finishperiod; t++)
                    {
                        Pattern p = new Pattern(sequence, data);
                        double cplt= CalcReducedcost(p, l, t, solution, data);
                        AddColumn(sequence, l, t, 0, data);
                    }

                }
                if (!(betapool[b].col2.pattern.Sequence.Contains(betapool[b].col1.pattern.LastPart)))
                {
                    int startperiod = Math.Max(betapool[b].col2.period - 5, 1);
                    int finishperiod = Math.Min(betapool[b].col2.period + 5, data.Tmax);
                    int l = betapool[b].col2.machine;
                    sequence = new List<int>(betapool[b].col2.pattern.Sequence);
                    sequence.Insert(0, betapool[b].col1.pattern.LastPart);
                    for (int t = startperiod; t <= finishperiod; t++)
                    {
                        Pattern p = new Pattern(sequence, data);
                        double cplt = CalcReducedcost(p, l, t, solution, data);
                        AddColumn(sequence, l, t, 0, data);
                    }
                }
            }
        }

        private void BackloggedParts(Solution solution, ProblemData data)
        {
            Dictionary<int, double> B_dict = new Dictionary<int, double>();
            //Get backlogged parts
            for (int i = 1; i <= data.Imax; i++)
            {
                double bc = 0;
                for (int t = 1; t <= data.Tmax; t++)
                {
                    bc = bc + solution.b[i, t] * data.BC_i[i];
                }
                if (bc > 0) B_dict.Add(i, bc);
            }
            int count = 0;
            int limit = (int)Math.Round((double)B_dict.Count * 3 / 4);
            foreach (var a in B_dict.OrderBy(x => x.Value))
            {
                if (count <= limit) B_dict.Remove(a.Key);

                count++;
            }
            List<int> B_list = new List<int>(B_dict.Keys.ToList());

            int itercount = 0;
            int columncount = 0;
            Random rnd = new Random();
            if (B_list.Count() > 0)
            {
                do
                {
                    int a = rnd.Next(0, B_list.Count - 1);
                    int i = B_list[a];
                    List<int> machines = data.GetCompatibleMachines(i);
                    if (machines.Count != 0)
                    {
                        int b = rnd.Next(0, machines.Count - 1);
                        int l = machines[b];
                        for (int t = 1; t <= data.Tmax; t++)
                        {
                            double cplt = CalcReducedcost(i, l, t, solution, data);
                            if (cplt < 0)
                            {
                                List<int> sequence = new List<int> { i };
                                if (AddColumn(sequence, l, t, 0, data))
                                {
                                    columncount++;
                                }
                            }
                        }
                    }
                    itercount++;

                } while (!(itercount > 300000 || columncount > 2000));
            }
            Console.WriteLine("Backlogged parts search: iteration count : " + itercount + " column count : " + columncount);
        }

        private void BackloggedPartsInjection(Solution solution, ProblemData data)
        {
            Dictionary<int, double> B_dict = new Dictionary<int, double>();
            //Get backlogged parts
            for (int i = 1; i <= data.Imax; i++)
            {
                double bc = 0;
                for (int t = 1; t <= data.Tmax; t++)
                {
                    bc = bc + solution.b[i, t] * data.BC_i[i];
                }
                if (bc > 0) B_dict.Add(i, bc);
            }
            int count = 0;
            int limit = (int)Math.Round((double)B_dict.Count * 3 / 4);
            foreach (var a in B_dict.OrderBy(x => x.Value))
            {
                if (count <= limit) B_dict.Remove(a.Key);

                count++;
            }
            List<int> B_list = new List<int>(B_dict.Keys.ToList());


            Console.WriteLine("total backlog part is : " + B_list.Count);
            int itercount = 0;
            int columncount = 0;
            Random rnd = new Random();
            if (B_list.Count > 0)
            {
                do
                {
                    int a = rnd.Next(0, B_list.Count - 1);
                    int i = B_list[a];
                    List<int> machines = data.GetCompatibleMachines(i);
                    if (machines.Count != 0)
                    {
                        int b = rnd.Next(0, machines.Count - 1);
                        int l = machines[b];

                        for (int t = 1; t <= data.Tmax; t++)
                        {
                            var alfacol = alfapool.Where(x => x.Value.start == 1 && x.Value.machine == l && x.Value.period == t).FirstOrDefault();
                            if (alfacol.Value != null)
                            {
                                var sequence = alfacol.Value.pattern.Sequence;

                                for (int p = 0; p < sequence.Count; p++)
                                {
                                    List<int> newsequence = new List<int>(sequence);
                                    newsequence.Insert(p, i);
                                    double cplt = 0;
                                    Pattern pat = new Pattern(newsequence, data);
                                    cplt = CalcReducedcost(pat, l, t, solution, data);
                                    if (cplt < 0)
                                    {
                                        if (AddColumn(newsequence, l, t, 0, data))
                                        {
                                            columncount++;
                                        }
                                    }
                                }
                            }
                            itercount++;
                        }
                    }
                } while (!(itercount > 300000 || columncount > 2500));
            }
            
            Console.WriteLine("Backlogged parts injection search: iteration count : " + itercount + " column count : " + columncount);
        }

        private void RandomParts(Solution solution, ProblemData data)
        {
            int itercount = 0;
            int columncount = 0;
            Random rnd = new Random();
            do
            {
                int i = rnd.Next(1, data.Imax);
                List<int> machines = data.GetCompatibleMachines(i);
                if (machines.Count != 0)
                {
                    int b = rnd.Next(0, machines.Count - 1);
                    int l = machines[b];
                    for (int t = 1; t <= data.Tmax; t++)
                    {
                        double cplt = CalcReducedcost(i, l, t, solution, data);
                        if (cplt < 0)
                        {
                            List<int> sequence = new List<int> { i };
                            cplt = CalcReducedcost(i, l, t, solution, data);
                            if (AddColumn(sequence, l, t, 0, data))
                            {
                                columncount++;
                            }
                        }
                        itercount++;
                    }
                }
                
            } while (!(itercount > 300000 || columncount > 500));
            Console.WriteLine("Random parts : iteration count : " + itercount + " column count : " + columncount);

        }

        public void BestPosition(Solution solution, ProblemData data)
        {
            for (int i = 1; i <= data.Imax; i++)
            {
                double reducedcost = 0;
                int machine = 0;
                int period = 0;
                for (int l = 1; l <= data.Lmax; l++)
                {
                    for (int t = 1; t <= data.Tmax; t++)
                    {
                        double cplt = CalcReducedcost(i, l, t, solution, data);
                        if (reducedcost > cplt)
                        {
                            reducedcost = cplt;
                            machine = l;
                            period = t;
                        }

                    }
                }

                List<int> sequence = new List<int> { i };
                AddColumn(sequence, machine, period, 0, data);
            }
        }

        public double CalcReducedcost(Pattern pat, int l, int t, Solution solution, ProblemData data)
        {
            double cplt = pat.ST * data.SC;
            cplt = cplt + solution.c7[l, t] + pat.ST * solution.c6[l, t];
            foreach (var seq in pat.Sequence)
            {
                cplt = cplt - solution.c5[seq, l, t] + solution.c9[seq, l, t];
            }
            if (t > 1)
            {
                int alfaindex = solution.alfamap[l, t - 1];
                if (alfaindex != 0)
                {
                    int lastpartindex = alfapool[alfaindex].pattern.LastPart;
                    double mu = Math.Min(solution.c6[l, t], solution.c6[l, t - 1]);

                    cplt = cplt + (data.SC + mu) * data.ST_ij[lastpartindex, pat.FirstPart];
                }
            }
            if (t < data.Tmax)
            {
                int alfaindex = solution.alfamap[l, t + 1];
                if (alfaindex != 0)
                {
                    int firstpartindex = alfapool[alfaindex].pattern.FirstPart;

                    double mu = Math.Min(solution.c6[l, t], solution.c6[l, t + 1]);

                    

                    cplt = cplt + (data.SC+mu) * data.ST_ij[pat.LastPart, firstpartindex];
                }
            }
            string sequencepat = string.Join("-", pat.Sequence.Select(num => num.ToString()));
            string colstring = sequencepat + "-" + l + "-" + t;
            if (lst_searched_column.ContainsKey(colstring))
            {
                lst_searched_column[colstring] += 1;
            }
            else
            {
                lst_searched_column.Add(colstring, 1);
            }
            return cplt;
        }

        public double CalcReducedcost(int i, int l, int t, Solution solution, ProblemData data)
        {
            double cplt = 0;
            cplt = cplt + solution.c7[l, t];
            cplt = cplt - solution.c5[i, l, t] + solution.c9[i, l, t];

            if (t > 1)
            {
                int alfaindex = solution.alfamap[l, t - 1];
                if (alfaindex != 0)
                {
                    int lastpartindex = alfapool[alfaindex].pattern.LastPart;
                    cplt = cplt + data.SC * data.ST_ij[lastpartindex, i];
                }

            }
            if (t < data.Tmax)
            {
                int alfaindex = solution.alfamap[l, t + 1];
                if (alfaindex != 0)
                {
                    int firstpartindex = alfapool[alfaindex].pattern.FirstPart;
                    cplt = cplt + data.SC * data.ST_ij[i, firstpartindex];
                }
            }
            string colstring = i + "-" + l + "-" + t;
            if (lst_searched_column.ContainsKey(colstring))
            {
                lst_searched_column[colstring] += 1;
            }
            else
            {
                lst_searched_column.Add(colstring, 1);
            }
            return cplt;
        }
    }

    public class Beta
    {
        public Column col1 { get; set; }
        public Column col2 { get; set; }
        public int setup { get; set; }
        public int start;
        public Beta(Column _col1, Column _col2, ProblemData data)
        {
            col1 = _col1;
            col2 = _col2;
            setup = data.ST_ij[col1.pattern.LastPart, col2.pattern.FirstPart];
        }
    }
}
