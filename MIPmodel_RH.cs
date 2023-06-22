using System;
using System.IO;
using Gurobi;

namespace RF_HEURISTICS
{
    public class MIPmodel_RH
    {

        public GRBVar[,] s;
        public GRBVar[,] b;
        public GRBVar[,,] q;
        public GRBVar[,,] S;
        public GRBVar[,,,] y;
        public GRBVar[,,] z;
        public GRBLinExpr[] c10, c11;
        public GRBLinExpr[,] c1, c2, c4, c12;
        public GRBLinExpr[,,] c3, c5, c16, c17, c18, c20, c22;
        public GRBLinExpr[,,,] c6, c9, c19, c21;
        public GRBConstr[] constr10, constr11;
        public GRBConstr[,] constr1, constr2, constr4, constr12;
        public GRBConstr[,,] constr3, constr5, constr8, constr16, constr17, constr18, constr20, constr22;
        public GRBConstr[,,,] constr6, constr7, constr9, constr19, constr21;
        public GRBLinExpr obj;
        public int Imax, Lmax, Tmax;
        GRBModel model;
        GRBEnv env;
        public string modeltype;
        public MIPmodel_RH(ProblemData data,Solution solution)
        {
            Imax = data.Imax;
            Lmax = data.Lmax;
            Tmax  = data.Tmax;
            env = new GRBEnv();
            model = new GRBModel(env);
            init_variables();
            init_constraints(data, solution);
            init_obj(data);
        }

        void init_variables()
        {
            s = new GRBVar[Imax + 1, Tmax + 1];
            b = new GRBVar[Imax + 1, Tmax + 1];
            q = new GRBVar[Imax + 1, Lmax + 1, Tmax + 1];
            S = new GRBVar[Imax + 1, Lmax + 1, Tmax + 1];
            z = new GRBVar[Imax + 1, Lmax + 1, Tmax + 1];
            y = new GRBVar[Imax + 1, Imax + 1, Lmax + 1, Tmax + 1];


            for (int i = 1; i <= Imax; i++)
            {
                
                for (int t = 0; t <= Tmax; t++)
                {
                    s[i, t] = model.AddVar(0, int.MaxValue, 0, GRB.CONTINUOUS, "s-" + i + "-" + t);
                    b[i, t] = model.AddVar(0, int.MaxValue, 0, GRB.CONTINUOUS, "b-" + i + "-" + t);
                    for (int l = 1; l <= Lmax; l++)
                    {
                        q[i, l, t] = model.AddVar(0, int.MaxValue, 0, GRB.CONTINUOUS, "q-" + i + "-" + l + "-" + t);
                        S[i, l, t] = model.AddVar(0, int.MaxValue, 0, GRB.INTEGER, "S-" + i + "-" + l + "-" + t);
                        z[i, l, t] = model.AddVar(0, 1, 0, GRB.BINARY, "z-" + i + "-" + l + "-" + t);
                        for (int j = 1; j <= Imax; j++)
                        {
                            y[i, j, l, t] = model.AddVar(0, 1, 0, GRB.BINARY, "y-" + i + "-" + j + "-" + l + "-" + t);
                        }
                    }
                }
            }
        }

        void init_constraints(ProblemData data,Solution solution)
        {
            c1 = new GRBLinExpr[Imax + 1, Tmax + 1];
            c2 = new GRBLinExpr[Lmax + 1, Tmax + 1];
            c3 = new GRBLinExpr[Imax + 1, Lmax + 1, Tmax + 1];
            c4 = new GRBLinExpr[Lmax + 1, Tmax + 1];
            c5 = new GRBLinExpr[Imax + 1, Lmax + 1, Tmax + 1];
            c6 = new GRBLinExpr[Imax + 1, Imax + 1, Imax + 1, Tmax + 1];
            c9 = new GRBLinExpr[Imax + 1, Imax + 1, Lmax + 1, Tmax + 1];
            c10 = new GRBLinExpr[Imax + 1];
            c11 = new GRBLinExpr[Imax + 1];
            c12 = new GRBLinExpr[Imax+1, Lmax + 1];

            c19 = new GRBLinExpr[Imax + 1, Imax + 1, Lmax + 1, Tmax + 1];
            c20 = new GRBLinExpr[Imax + 1, Lmax + 1, Tmax + 1];
            c21 = new GRBLinExpr[Imax + 1, Imax + 1, Lmax + 1, Tmax + 1];
            c22 = new GRBLinExpr[Imax + 1, Lmax + 1, Tmax + 1];


            constr1 = new GRBConstr[Imax + 1, Tmax + 1];
            constr2 = new GRBConstr[Lmax + 1, Tmax + 1];
            constr3 = new GRBConstr[Imax + 1, Lmax + 1, Tmax + 1];
            constr4 = new GRBConstr[Lmax + 1, Tmax + 1];
            constr5 = new GRBConstr[Imax + 1, Lmax + 1, Tmax + 1];
            constr6 = new GRBConstr[Imax + 1, Imax + 1, Lmax + 1, Tmax + 1];
            constr9 = new GRBConstr[Imax + 1, Imax + 1, Lmax + 1, Tmax + 1];
            constr10 = new GRBConstr[Imax + 1];
            constr11 = new GRBConstr[Imax + 1];
            constr12 = new GRBConstr[Imax + 1, Lmax + 1];

            constr19 = new GRBConstr[Imax + 1, Imax + 1, Lmax + 1, Tmax + 1];
            constr20 = new GRBConstr[Imax + 1, Lmax + 1, Tmax + 1];
            constr21 = new GRBConstr[Imax + 1, Imax + 1, Lmax + 1, Tmax + 1];
            constr22 = new GRBConstr[Imax + 1, Lmax + 1, Tmax + 1];

            //c1 - Inventory balance
            for (int i = 1; i <= Imax; i++)
            {
                for (int t = 1; t <= Tmax; t++)
                {
                    c1[i, t] = 0.0;
                    c1[i, t].AddTerm(1, s[i, t - 1]);
                    c1[i, t].AddTerm(-1, b[i, t - 1]);
                    c1[i, t].AddTerm(1, b[i, t]);
                    c1[i, t].AddTerm(-1, s[i, t]);
                    for (int l = 1; l <= Lmax; l++)
                    {
                        c1[i, t].AddTerm(1, q[i, l, t]);
                    }
                    constr1[i, t] = model.AddConstr(c1[i, t], GRB.EQUAL, data.D_it[i, t], "c1-" + i + "-" + t);
                }
            }

            //c2 - Capacity constraint
            for (int l = 1; l <= Lmax; l++)
            {
                for (int t = 1; t <= Tmax; t++)
                {
                    c2[l, t] = 0.0;

                    for (int i = 1; i <= Imax; i++)
                    {
                        c2[l, t].AddTerm(data.CT_i[i], q[i, l, t]);
                        for (int j = 1; j <= Imax; j++)
                        {
                            c2[l, t].AddTerm(data.ST_ij[i, j], y[i, j, l, t]);
                        }

                    }
                    constr2[l, t] = model.AddConstr(c2[l, t], GRB.LESS_EQUAL, data.Tcap, "c2-" + l + "-" + t);

                }
            }

            //c3 - Setup constraint (production possible if carried over or there is a setup to a product)
            for (int l = 1; l <= Lmax; l++)
            {
                for (int t = 1; t <= Tmax; t++)
                {
                    for (int i = 1; i <= Imax; i++)
                    {
                        c3[i, l, t] = 0.0;

                        for (int j = 1; j <= Imax; j++)
                        {
                            if (i != j)
                            {
                                c3[i, l, t].AddTerm(-data.term3[i, l, t], y[j, i, l, t]);
                            }
                        }
                        c3[i, l, t].AddTerm(-data.term3[i, l, t], z[i, l, t - 1]);
                        c3[i, l, t].AddTerm(1, q[i, l, t]);

                        constr3[i, l, t] = model.AddConstr(c3[i, l, t], GRB.LESS_EQUAL, 0.0, "c3-" + i + "-" + l + "-" + t);

                    }
                }
            }

            //c4 - Only one setup carry over from previous period
            for (int l = 1; l <= Lmax; l++)
            {
                for (int t = 1; t <= Tmax; t++)
                {
                    c4[l, t] = 0.0;
                    for (int i = 1; i <= Imax; i++)
                    {
                        c4[l, t].AddTerm(1, z[i, l, t]);
                    }
                    constr4[l, t] = model.AddConstr(c4[l, t], GRB.EQUAL, 1, "c4-" + l + "-" + t);

                }
            }

            //c5 - Setup flow balance
            for (int l = 1; l <= Lmax; l++)
            {
                for (int i = 1; i <= Imax; i++)
                {
                    for (int t = 1; t <= Tmax; t++)
                    {
                        c5[i, l, t] = 0.0;
                        for (int j = 1; j <= Imax; j++)
                        {
                            if (j != i)
                            {
                                c5[i, l, t].AddTerm(1, y[j, i, l, t]);
                                c5[i, l, t].AddTerm(-1, y[i, j, l, t]);
                            }
                        }
                        c5[i, l, t].AddTerm(1, z[i, l, t - 1]);
                        c5[i, l, t].AddTerm(-1, z[i, l, t]);
                        constr5[i, l, t] = model.AddConstr(c5[i, l, t], GRB.EQUAL, 0, "c5-" + i + "-" + l + "-" + t);

                    }
                }
            }

            //c6 - Setup cycle elimination constraint
            for (int i = 1; i <= Imax; i++)
            {
                for (int j = 1; j <= Imax; j++)
                {
                    for (int l = 1; l <= Lmax; l++)
                    {
                        for (int t = 1; t <= Tmax; t++)
                        {
                            c6[i, j, l, t] = 0.0;
                            c6[i, j, l, t].AddTerm(1, S[j, l, t]);
                            c6[i, j, l, t].AddTerm(-1, S[i, l, t]);
                            c6[i, j, l, t].AddTerm(-Imax, y[i, j, l, t]);
                            constr6[i, j, l, t] = model.AddConstr(c6[i, j, l, t], GRB.GREATER_EQUAL, 1 - Imax, "c6-" + i + "-" + j + "-" + l + "-" + t);
                        }
                    }
                }
            }

            //c9 canceled - Machine availability constraint not used!

            //c10-11 - Initial backlog and inventory
            for (int i = 1; i <= Imax; i++)
            {
                c10[i] = 0.0;
                c11[i] = 0.0;
                c10[i].AddTerm(1, s[i, 0]);
                c11[i].AddTerm(1, b[i, 0]);
                constr10[i] = model.AddConstr(c10[i], GRB.EQUAL, 0.0, "c10-" + i);
                constr11[i] = model.AddConstr(c11[i], GRB.EQUAL, 0.0, "c11-" + i);

            }

            //c12 - initial setup constraint
            /*
            for (int l = 1; l <= Lmax; l++)
            {
                for(int i = 1; i <= Imax; i++)
                {
                    c12[i, l] = 0.0;
                    c12[i, l].AddTerm(1, z[i, l, 0]);
                    if (data.Z0[l] != i)
                    {
                        constr12[i, l] = model.AddConstr(c12[i, l], GRB.EQUAL, 0, "c12-" + i + "-" + l);
                    }
                    else
                    {
                        constr12[i, l] = model.AddConstr(c12[i, l], GRB.EQUAL, 1, "c12-" + i + "-" + l);
                    }
                }
            }*/
            
            //c19-21


            for (int i = 1; i <= Imax; i++)
            {
                for (int j = 1; j <= Imax; j++)
                {
                        for (int l = 1; l <= Lmax; l++)
                        {
                            for (int t = 1; t <= Tmax; t++)
                            {
                                c19[i, j, l, t] = 0.0;
                                c19[i, j, l, t].AddTerm(1 - solution.x_t[t] * solution.x_m[l], y[i, j, l, t]);
                                constr19[i, j, l, t] = model.AddConstr(c19[i, j, l, t], GRB.EQUAL, (1 - solution.x_m[l] * solution.x_t[t]) * solution.Y_bar[i, j, l, t], "c19-" + i + "-" + j + "-" + l + "-" + t);

                                c21[i, j, l, t] = 0.0;
                                c21[i, j, l, t].AddTerm(1, y[i, j, l, t]);
                                constr21[i, j, l, t] = model.AddConstr(c21[i, j, l, t], GRB.LESS_EQUAL, solution.IL_bar_il[i, l], "c21-" + i + "-" + j + "-" + l + "-" + t);
                            }
                        }
                    

                }
            }


            //c20-22

            for (int i = 1; i <= Imax; i++)
            {
                for (int l = 1; l <= Lmax; l++)
                {
                    for (int t = 1; t <= Tmax; t++)
                    {
                        c20[i, l, t] = 0.0;
                        c20[i, l, t].AddTerm(1 - solution.x_t[t] * solution.x_m[l], z[i, l, t]);
                        constr20[i, l, t] = model.AddConstr(c20[i, l, t], GRB.EQUAL, (1 - solution.x_m[l] * solution.x_t[t]) * solution.Z_bar[i, l, t], "c20-" + i + "-" + l + "-" + t);

                        c22[i, l, t] = 0.0;
                        c22[i, l, t].AddTerm(1, z[i, l, t]);
                        constr22[i, l, t] = model.AddConstr(c22[i, l, t], GRB.LESS_EQUAL, solution.IL_bar_il[i, l], "c22-" + i + "-" + l + "-" + t);
                    }
                }
            }

        }

        void init_obj(ProblemData data)
        {
            obj = 0.0;
            for (int i = 1; i <= Imax; i++)
            {
                for (int t = 1; t <= Tmax; t++)
                {
                    obj.AddTerm(data.HC_i[i], s[i, t]);
                    obj.AddTerm(data.BC_i[i], b[i, t]);
                    for (int l = 1; l <= Lmax; l++)
                    {
                        for (int j = 1; j <= Imax; j++)
                        {
                            obj.AddTerm(data.ST_ij[i, j]*data.SC, y[i, j, l, t]);
                        }

                        obj.AddTerm(data.PC_l[l]*data.CT_i[i] , q[i, l, t]);

                    }
                }
            }
            model.SetObjective(obj, GRB.MINIMIZE);
        }

        void getsolutions(Solution solution, ProblemData data)
        {
            solution.productioncost = 0;
            solution.setupcost = 0;
            solution.inventorycost = 0;
            solution.backlogcost = 0;
            solution.totalcost = 0;
            for(int i = 1; i <= Imax; i++)
            {
                for(int l = 1; l <= Lmax; l++)
                {
                   // if (model.GetVarByName("z-" + i + "-" + l + "-" + 0).X>0.1) Console.WriteLine(i + "-" + l + "-" + model.GetVarByName("z-" + i + "-" + l + "-" + 0).X);
                    for(int t = 1; t <= Tmax; t++)
                    {
                        solution.q_ilt[i, l, t] = model.GetVarByName("q-" + i + "-" + l + "-" + t).X;
                        solution.productioncost += solution.q_ilt[i, l, t] * data.PC_l[l] * data.CT_i[i];
                        solution.Z_bar[i, l, t] = model.GetVarByName("z-" + i + "-" + l + "-" + t).X;
                        for(int j = 1; j <= Imax; j++)
                        {
                            solution.Y_bar[i, j, l, t] = model.GetVarByName("y-" + i + "-" + j + "-" + l + "-" + t).X;
                            solution.setupcost += solution.Y_bar[i, j, l, t] * data.ST_ij[i, j] * data.SC;
                        }
                    }
                }
            }

            for(int i = 1; i <= Imax; i++)
            {
                for(int t = 1; t <= Tmax; t++)
                {
                    solution.b[i, t] = model.GetVarByName("b-" + i + "-" + t).X;
                    solution.s[i, t] = model.GetVarByName("s-" + i + "-" + t).X;
                    solution.inventorycost += solution.s[i, t] * data.HC_i[i];
                    solution.backlogcost += solution.b[i, t] * data.BC_i[i];
                    
                }
            }
            solution.totalcost = solution.inventorycost + solution.backlogcost + solution.productioncost + solution.setupcost;
            solution.MIPGap = model.MIPGap;
            solution.obj = model.ObjVal;

        }

        public void solve(Solution solution, ProblemData data,int TL)
        {
            model.Parameters.TimeLimit = TL;
            if (modeltype == "RH") model.Parameters.TimeLimit = TL;
            model.Parameters.MIPGap = 0.001;
            model.Parameters.MIPFocus = 1;
            model.Update();

            model.Optimize();

            int status = model.Status;

            // Do IIS
           
            if(status == GRB.Status.INF_OR_UNBD || status == GRB.Status.INFEASIBLE)
            {
                 /*
                Console.WriteLine("The model is infeasible; computing IIS");
               
               model.ComputeIIS();
               // Console.WriteLine("\nThe following constraint(s) "
                  //  + "cannot be satisfied:");

                int counter = 0;
                for(int i = 1; i <= data.Imax; i++)
                {
                    if (solution.IL_bar_il[i, 4] == 1)
                    {
                       // Console.WriteLine("part " + i);
                        counter++;
                    }
                }
              //  Console.WriteLine(counter);

                foreach (GRBConstr c in model.GetConstrs())
                {
                    if (c.IISConstr == 1)
                    {
                        Console.WriteLine(c.ConstrName);
                    }
                }
                */
            }
            else
            {
                try
                {
                    getsolutions(solution, data);
                }
                catch
                {
                    solution.totalcost = 0;
                    solution.setupcost = 0;
                    solution.backlogcost = 0;
                    solution.inventorycost = 0;
                    solution.productioncost = 0;
                    solution.MIPGap = 0;
                }

            }


        }

        public void updatemodel(Solution solution, ProblemData data)
        {
            if (modeltype == "RH")
            {

                //relax variables for rh
                for (int t = 1; t <= Tmax; t++)
                {
                    for (int i = 1; i <= Imax; i++)
                    {
                        for (int l = 1; l <= Lmax; l++)
                        {
                            if (t > solution.te)
                            {
                                z[i, l, t].VType = GRB.CONTINUOUS;
                                for (int j = 1; j <= Imax; j++)
                                {
                                    y[i, j, l, t].VType = GRB.CONTINUOUS;
                                }
                            }
                            else
                            {
                                z[i, l, t].VType = GRB.BINARY;
                                for (int j = 1; j <= Imax; j++)
                                {
                                    y[i, j, l, t].VType = GRB.BINARY;
                                }
                            }
                        }
                    }
                }
                // fix variables for rh
                for(int t = 1; t <= Tmax; t++)
                {
                    if (t < solution.ts)
                    {
                        solution.x_t[t] = 0;
                    }
                    else
                    {
                        solution.x_t[t] = 1;
                    }
                }

                for(int l = 1; l <= Lmax; l++)
                {
                    solution.x_m[l] = 1;
                }
                model.Update();
                //Change constraints 19-21
                for (int i = 1; i <= Imax; i++)
                {
                    for (int j = 1; j <= Imax; j++)
                    {
                        for (int l = 1; l <= Lmax; l++)
                        {
                            for (int t = 1; t <= Tmax; t++)
                            {
                                model.Remove(model.GetConstrByName("c19-" + i + "-" + j + "-" + l + "-" + t));
                                c19[i, j, l, t] = 0.0;
                                c19[i, j, l, t].AddTerm(1 - solution.x_t[t] * solution.x_m[l], y[i, j, l, t]);
                                constr19[i, j, l, t] = model.AddConstr(c19[i, j, l, t], GRB.EQUAL, (1 - solution.x_m[l] * solution.x_t[t]) * solution.Y_bar[i, j, l, t], "c19-" + i + "-" + j + "-" + l + "-" + t);
                                model.Remove(model.GetConstrByName("c21-" + i + "-" + j + "-" + l + "-" + t));
                                c21[i, j, l, t] = 0.0;
                                c21[i, j, l, t].AddTerm(1, y[i, j, l, t]);
                                constr21[i, j, l, t] = model.AddConstr(c21[i, j, l, t], GRB.LESS_EQUAL, solution.IL_bar_il[i, l], "c21-" + i + "-" + j + "-" + l + "-" + t);
                            }
                        }
                    }
                }

                //c20-22

                for (int i = 1; i <= Imax; i++)
                {
                    for (int l = 1; l <= Lmax; l++)
                    {
                        for (int t = 1; t <= Tmax; t++)
                        {
                            model.Remove(model.GetConstrByName("c20-" + i + "-" + l + "-" + t));
                            c20[i, l, t] = 0.0;
                            c20[i, l, t].AddTerm(1 - solution.x_t[t] * solution.x_m[l], z[i, l, t]);
                            constr20[i, l, t] = model.AddConstr(c20[i, l, t], GRB.EQUAL, (1 - solution.x_m[l] * solution.x_t[t]) * solution.Z_bar[i, l, t], "c20-" + i + "-" + l + "-" + t);

                            model.Remove(model.GetConstrByName("c22-" + i + "-" + l + "-" + t));
                            c22[i, l, t] = 0.0;
                            c22[i, l, t].AddTerm(1, z[i, l, t]);
                            constr22[i, l, t] = model.AddConstr(c22[i, l, t], GRB.LESS_EQUAL, solution.IL_bar_il[i, l], "c22-" + i + "-" + l + "-" + t);
                        }
                    }
                }
                model.Update();

            }
            else if(modeltype =="HDFO")
            {


                //Convert model variables to binary
                for (int t = 1; t <= Tmax; t++)
                {
                    for (int i = 1; i <= Imax; i++)
                    {
                        for (int l = 1; l <= Lmax; l++)
                        {

                            z[i, l, t].VType = GRB.BINARY;
                            for (int j = 1; j <= Imax; j++)
                            {
                                y[i, j, l, t].VType = GRB.BINARY;
                            }
                        }
                    }
                }
                model.Update();
                //Change constraints 19-21
                for (int i = 1; i <= Imax; i++)
                {
                    for (int j = 1; j <= Imax; j++)
                    {
                        for (int l = 1; l <= Lmax; l++)
                        {
                            for (int t = 1; t <= Tmax; t++)
                            {
                                model.Remove(model.GetConstrByName("c19-" + i + "-" + j + "-" + l + "-" + t));
                                c19[i, j, l, t] = 0.0;
                                c19[i, j, l, t].AddTerm(1 - solution.x_t[t] * solution.x_m[l], y[i, j, l, t]);
                                constr19[i, j, l, t] = model.AddConstr(c19[i, j, l, t], GRB.EQUAL, (1 - solution.x_m[l] * solution.x_t[t]) * solution.Y_bar[i, j, l, t], "c19-" + i + "-" + j + "-" + l + "-" + t);
                                model.Remove(model.GetConstrByName("c21-" + i + "-" + j + "-" + l + "-" + t));
                                c21[i, j, l, t] = 0.0;
                                c21[i, j, l, t].AddTerm(1, y[i, j, l, t]);
                                constr21[i, j, l, t] = model.AddConstr(c21[i, j, l, t], GRB.LESS_EQUAL, solution.IL_bar_il[i, l], "c21-" + i + "-" + j + "-" + l + "-" + t);
                            }
                        }
                    }
                }

                //c20-22

                for (int i = 1; i <= Imax; i++)
                {
                    for (int l = 1; l <= Lmax; l++)
                    {
                        for (int t = 1; t <= Tmax; t++)
                        {
                            model.Remove(model.GetConstrByName("c20-" + i + "-" + l + "-" + t));
                            c20[i, l, t] = 0.0;
                            c20[i, l, t].AddTerm(1 - solution.x_t[t] * solution.x_m[l], z[i, l, t]);
                            constr20[i, l, t] = model.AddConstr(c20[i, l, t], GRB.EQUAL, (1 - solution.x_m[l] * solution.x_t[t]) * solution.Z_bar[i, l, t], "c20-" + i + "-" + l + "-" + t);

                            model.Remove(model.GetConstrByName("c22-" + i + "-" + l + "-" + t));
                            c22[i, l, t] = 0.0;
                            c22[i, l, t].AddTerm(1, z[i, l, t]);
                            constr22[i, l, t] = model.AddConstr(c22[i, l, t], GRB.LESS_EQUAL, solution.IL_bar_il[i, l], "c22-" + i + "-" + l + "-" + t);
                        }
                    }
                }
                model.Update();

            }
            else if(modeltype == "Exact")
            {
                for(int i = 1; i <= data.Imax; i++)
                {
                    for(int l = 1; l <= data.Lmax; l++)
                    {
                        solution.IL_bar_il[i, l] = data.IL_il[i, l];
                    }
                }

                for(int t = 1; t <= data.Tmax; t++)
                {
                    solution.x_t[t] = 1;
                }
                for(int l = 1; l <= data.Lmax; l++)
                {
                    solution.x_m[l] = 1;
                }

                //Convert model variables to binary
                for (int t = 1; t <= Tmax; t++)
                {
                    for (int i = 1; i <= Imax; i++)
                    {
                        for (int l = 1; l <= Lmax; l++)
                        {

                            z[i, l, t].VType = GRB.BINARY;
                            for (int j = 1; j <= Imax; j++)
                            {
                                y[i, j, l, t].VType = GRB.BINARY;
                            }
                        }
                    }
                }
                model.Update();
                //Change constraints 19-21
                for (int i = 1; i <= Imax; i++)
                {
                    for (int j = 1; j <= Imax; j++)
                    {
                        for (int l = 1; l <= Lmax; l++)
                        {
                            for (int t = 1; t <= Tmax; t++)
                            {
                                model.Remove(model.GetConstrByName("c19-" + i + "-" + j + "-" + l + "-" + t));
                                c19[i, j, l, t] = 0.0;
                                c19[i, j, l, t].AddTerm(1 - solution.x_t[t] * solution.x_m[l], y[i, j, l, t]);
                                constr19[i, j, l, t] = model.AddConstr(c19[i, j, l, t], GRB.EQUAL, (1 - solution.x_m[l] * solution.x_t[t]) * solution.Y_bar[i, j, l, t], "c19-" + i + "-" + j + "-" + l + "-" + t);
                                model.Remove(model.GetConstrByName("c21-" + i + "-" + j + "-" + l + "-" + t));
                                c21[i, j, l, t] = 0.0;
                                c21[i, j, l, t].AddTerm(1, y[i, j, l, t]);
                                constr21[i, j, l, t] = model.AddConstr(c21[i, j, l, t], GRB.LESS_EQUAL, solution.IL_bar_il[i, l], "c21-" + i + "-" + j + "-" + l + "-" + t);
                            }
                        }
                    }
                }

                //c20-22

                for (int i = 1; i <= Imax; i++)
                {
                    for (int l = 1; l <= Lmax; l++)
                    {
                        for (int t = 1; t <= Tmax; t++)
                        {
                            model.Remove(model.GetConstrByName("c20-" + i + "-" + l + "-" + t));
                            c20[i, l, t] = 0.0;
                            c20[i, l, t].AddTerm(1 - solution.x_t[t] * solution.x_m[l], z[i, l, t]);
                            constr20[i, l, t] = model.AddConstr(c20[i, l, t], GRB.EQUAL, (1 - solution.x_m[l] * solution.x_t[t]) * solution.Z_bar[i, l, t], "c20-" + i + "-" + l + "-" + t);

                            model.Remove(model.GetConstrByName("c22-" + i + "-" + l + "-" + t));
                            c22[i, l, t] = 0.0;
                            c22[i, l, t].AddTerm(1, z[i, l, t]);
                            constr22[i, l, t] = model.AddConstr(c22[i, l, t], GRB.LESS_EQUAL, solution.IL_bar_il[i, l], "c22-" + i + "-" + l + "-" + t);
                        }
                    }
                }
                model.Update();
            }
            


        }


    }
}
