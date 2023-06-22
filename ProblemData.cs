using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;

namespace RF_HEURISTICS
{
    public class ProblemData
    {
        //Problem specific parameters
        public int Lmax;
        public int Imax;
        public int Tmax;
        public int Mmax;
        public double SC;
        public double[] HC_i;
        public double[] BC_i;
        public double[] CT_i;
        public double[] PC_l;
        public int[,] ST_ij;
        public int[,] D_it;
        public int Tcap;
        public int[,] IM_im;
        public int[,] ML_ml;
        public int[,] IL_il;
        public int[] Z0;
        //Calculated parameters

        public int[] D_prime;
        public double[,,] term3;
        public double dit_sum;
        public int[] A_j;
        public double[] FI_m;
        public int[] p_j;
        public double[,] Theta_il;
        public List<int>[] sigma_j;
        public List<int>[] phi_j;

        public string filename;

        public ProblemData(string path)
        {
            string[] xxx = path.Split("/");
            filename = xxx[xxx.Length - 1];

            read_data(path);
        }

        public void read_data(string path)
        {

            string[] lines = File.ReadAllLines(path);

            Lmax = int.Parse(lines[0]);
            Imax = int.Parse(lines[1]);
            Tmax = int.Parse(lines[2]);
            Mmax = int.Parse(lines[3]);
            Tcap = int.Parse(lines[4]);
            SC = double.Parse(lines[5]);

            string[] str_HC_i = lines[6].Split(" ");
            HC_i = new double[Imax + 1];
            for(int i = 1; i <= Imax; i++)
            {
                HC_i[i] = double.Parse(str_HC_i[i - 1]);
            }

            string[] str_BC_i = lines[7].Split(" ");
            BC_i = new double[Imax + 1];
            for (int i = 1; i <= Imax; i++)
            {
                BC_i[i] = double.Parse(str_BC_i[i - 1]);
            }

            string[] str_CT_i = lines[8].Split(" ");
            CT_i = new double[Imax + 1];
            for (int i = 1; i <= Imax; i++)
            {
                CT_i[i] = double.Parse(str_CT_i[i - 1]);
            }

            ST_ij = new int[Imax + 1, Imax + 1];
            for (int i = 1; i <= Imax; i++)
            {
                string[] str_ST_ij = lines[8 + i].Split(" ");

                for (int j = 1; j <= Imax; j++)
                {
                    ST_ij[i, j] = int.Parse(str_ST_ij[j - 1]);
                }
            }

            D_it = new int[Imax + 1, Tmax + 1];
            for (int i = 1; i <= Imax; i++)
            {
                string[] str_D_it = lines[8 + Imax + i].Split(" ");

                for (int t = 1; t <= Tmax; t++)
                {
                    D_it[i, t] = int.Parse(str_D_it[t - 1]);
                }
            }


            IL_il = new int[Imax + 1, Lmax + 1];
            for (int i = 1; i <= Imax; i++)
            {
                string[] str_IL_il = lines[8 + 2*Imax + i].Split(" ");

                for (int l = 1; l <= Lmax; l++)
                {
                    IL_il[i, l] = int.Parse(str_IL_il[l - 1]);
                }
            }

            PC_l = new double[Lmax + 1];
            string[] str_PC_l = lines[9 + 3 * Imax].Split(" ");

            for (int l = 1; l <= Lmax; l++)
            {
                PC_l[l] = double.Parse(str_PC_l[l - 1]);
            }

            ML_ml = new int[Mmax + 1, Lmax + 1];
            for (int m = 1; m <= Mmax; m++)
            {
                string[] str_ML_ml = lines[9 + 3 * Imax + m].Split(" ");

                for (int l = 1; l <= Lmax; l++)
                {
                    ML_ml[m, l] = int.Parse(str_ML_ml[l - 1]);
                }
            }

            IM_im = new int[Imax + 1, Mmax + 1];
            for (int i = 1; i <= Imax; i++)
            {
                string[] str_IM_im = lines[9 + 3 * Imax + Mmax + i].Split(" ");

                for (int m = 1; m <= Mmax; m++)
                {
                    IM_im[i, m] = int.Parse(str_IM_im[m - 1]);
                }
            }

            Z0 = new int[Lmax + 1];
            string[] str_Z0 = lines[10 + 4 * Imax + Mmax].Split(" ");

            for (int l = 1; l <= Lmax; l++)
            {
                Z0[l] = int.Parse(str_Z0[l - 1]);
            }

            D_prime = new int[Imax + 1];
            for(int i = 1; i <= Imax; i++)
            {
                for(int t = 1; t <= Tmax; t++)
                {
                    D_prime[i] = D_prime[i] + D_it[i, t];
                    dit_sum = dit_sum + D_it[i, t];
                }
            }
            


            //term3

            term3 = new double[Imax + 1, Lmax + 1, Tmax + 1];

            for(int i = 1; i <= Imax; i++)
            {
                for(int l = 1; l <= Lmax; l++)
                {
                    for (int t = 1; t <= Tmax; t++)
                    {
                        double a = Tcap / CT_i[i];
                        double b = D_prime[i];

                        term3[i, l, t] = Math.Min(a, b);
                    }
                }
            }


            FI_m = new double[Lmax + 1];
            p_j = new int[Imax + 1];
            Theta_il = new double[Imax + 1, Lmax + 1];
            //FI_m
            for(int l = 1; l <= Lmax; l++)
            {
                for(int i = 1; i <= Imax; i++)
                {
                    FI_m[l] = FI_m[l] + IL_il[i, l];
                }
                FI_m[l] = FI_m[l] / Imax;
            }


            //p_j

            for(int i = 1; i <= Imax; i++)
            {
                p_j[i] = int.Parse(Math.Ceiling(D_prime[i] / (Tcap / CT_i[i])).ToString());
            }

            //Theta_il A_j
            Theta_il = new double[Imax + 1, Lmax + 1];
            A_j = new int[Imax + 1];
            for(int i = 1; i <= Imax; i++)
            {
                double denominator = 0;
                for(int l = 1; l <= Lmax; l++)
                {
                    if (IL_il[i, l] == 1)
                    {
                        denominator += 1 / FI_m[l];
                        A_j[i] = A_j[i] + 1;
                    }
                }
                double flag = 0;
                for(int l = 1; l <= Lmax; l++)
                {
                    if (IL_il[i, l] == 1)
                    {
                        Theta_il[i, l] = flag + (1 / FI_m[l])/denominator;
                        flag = Theta_il[i, l];
                    }
                }
            }

            //sigma_j
            sigma_j = new List<int>[Imax + 1];
            phi_j = new List<int>[Imax + 1];
            for(int i = 1; i <= Imax; i++)
            {
                double mincost = int.MaxValue;
                sigma_j[i] = new List<int>();
                phi_j[i] = new List<int>();
                for(int l = 1; l <= Lmax; l++)
                {
                    if (PC_l[l]<mincost && IL_il[i, l] == 1)
                    {
                        mincost = PC_l[l];
                    }
                }

                for(int l = 1; l <= Lmax; l++)
                {
                    if (mincost == PC_l[l] && IL_il[i, l] == 1)
                    {
                        sigma_j[i].Add(l);
                    }
                    if (IL_il[i, l] == 1)
                    {
                        phi_j[i].Add(l);
                    }
                }
            }
            
        }

        public List<int> GetCompatibleMachines(int part)
        {
            List<int> compmachines = new List<int>();

            for (int l = 1; l <= Lmax; l++)
            {
                if (IL_il[part, l] == 1)
                {
                    if (!(compmachines.Contains(l))) compmachines.Add(l);
                }
            }

            return compmachines;
        }

        public List<int> GetCompatibleMachines(List<int> pattern)
        {
            List<int> compmachines = new List<int>();


            for(int l = 1; l <= Lmax; l++)
            {
                bool boolflag = false;
                foreach(var i in pattern)
                {
                    if (IL_il[i, l] == 1)
                    {
                        boolflag = true;
                    }
                    else
                    {
                        boolflag = false;
                        goto flag;
                    }
                }
            flag:;
                if(boolflag == true)
                {
                    if (!(compmachines.Contains(l))) compmachines.Add(l);
                }
            }
            return compmachines;
        }

    }
}
