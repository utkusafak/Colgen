using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.IO;
using Gurobi;
namespace RF_HEURISTICS.COLGEN
{
    public class MasterMIP
    {

        GRBModel model;
        GRBEnv env;
        public GRBVar[] alfa, beta;
        public GRBVar[,,] q;
        public GRBVar[,] s, b, wf, t_plus, t_minus;
        public GRBVar[,] stco;
        public GRBLinExpr expr;
        public List<GRBConstr> constr2;
        public List<GRBConstr> constr5;
        public List<GRBConstr> constr6;
        public List<GRBConstr> constr7;
        public List<GRBConstr> constr9;
        public List<GRBConstr> constr11;
        public List<GRBConstr> constr12;
        public List<GRBConstr> constr13;
        public bool isRelaxed;

        public MasterMIP()
        {

        }


        public void initVariables(ColumnPool columnpool, ProblemData data)
        {
            //Define alfa from the column pool

            alfa = new GRBVar[columnpool.alfapool.Count + 1];
            beta = new GRBVar[columnpool.betapool.Count + 1];
            q = new GRBVar[data.Imax + 1, data.Lmax + 1, data.Tmax + 1];
            s = new GRBVar[data.Imax + 1, data.Tmax + 1];
            b = new GRBVar[data.Imax + 1, data.Tmax + 1];
            wf = new GRBVar[data.Lmax + 1, data.Tmax + 1];
            t_plus = new GRBVar[data.Lmax + 1, data.Tmax + 1];
            t_minus = new GRBVar[data.Lmax + 1, data.Tmax + 1];

            stco = new GRBVar[data.Lmax + 1, data.Tmax + 1];

            //alfa pool
            foreach (var a in columnpool.alfapool)
            {

                if (isRelaxed == false)
                {
                    alfa[a.Key] = model.AddVar(0, 1, 0, GRB.BINARY, a.Value.id.ToString());
                }
                else
                {
                    alfa[a.Key] = model.AddVar(0, 1, 0, GRB.CONTINUOUS, a.Value.id.ToString());
                }
            }

            //beta pool
            foreach (var b in columnpool.betapool)
            {
                if (isRelaxed == false)
                {
                    beta[b.Key] = model.AddVar(0, 1, 0, GRB.BINARY, "beta-" + b.Key);
                }
                else
                {
                    beta[b.Key] = model.AddVar(0, 1, 0.0, GRB.CONTINUOUS, "beta-" + b.Key);
                }
            }

            //q ilt
            for (int i = 1; i <= data.Imax; i++)
            {
                for (int l = 1; l <= data.Lmax; l++)
                {
                    for (int t = 1; t <= data.Tmax; t++)
                    {
                        q[i, l, t] = model.AddVar(0, 1, 0, GRB.CONTINUOUS, "q-" + i + "-" + l + "-" + t);
                    }
                }
            }

            //s b it
            for (int i = 1; i <= data.Imax; i++)
            {
                for (int t = 0; t <= data.Tmax; t++)
                {
                    s[i, t] = model.AddVar(0.0, GRB.MAXINT, 0.0, GRB.CONTINUOUS, "s-" + i + "-" + t);
                    b[i, t] = model.AddVar(0.0, GRB.MAXINT, 0.0, GRB.CONTINUOUS, "b-" + i + "-" + t);

                }
            }

            //wf t_plus t_minus
            for (int l = 1; l <= data.Lmax; l++)
            {
                for (int t = 1; t <= data.Tmax; t++)
                {
                    if (isRelaxed == false)
                    {
                        wf[l, t] = model.AddVar(0, 1, 0, GRB.BINARY, "wf-" + l + "-" + t);
                        stco[l, t] = model.AddVar(0, 1, 0, GRB.BINARY, "stco-" + l + "-" + t);
                    }
                    else
                    {
                        wf[l, t] = model.AddVar(0, 1, 0, GRB.CONTINUOUS, "wf-" + l + "-" + t);
                        stco[l, t] = model.AddVar(0, 1, 0, GRB.CONTINUOUS, "stco-" + l + "-" + t);

                    }
                    t_plus[l, t] = model.AddVar(0, data.Tcap, 0, GRB.CONTINUOUS, "t_plus-" + l + "-" + t);
                    t_minus[l, t] = model.AddVar(0, data.Tcap, 0, GRB.CONTINUOUS, "t_minus-" + l + "-" + t);
                }
            }

        }

        public void initConstraints(ColumnPool columnPool, ProblemData data, Solution solution)
        {
            expr = new GRBLinExpr();
            constr2 = new List<GRBConstr>();
            constr5 = new List<GRBConstr>();
            constr6 = new List<GRBConstr>();
            constr7 = new List<GRBConstr>();
            constr9 = new List<GRBConstr>();
            constr11 = new List<GRBConstr>();
            constr12 = new List<GRBConstr>();
            constr13 = new List<GRBConstr>();
            //Eq2a,2b,3a,3b,4a,4b
            for (int i = 1; i <= data.Imax; i++)
            {
                expr = 0;
                expr.AddTerm(1, s[i, 0]);
                model.AddConstr(expr, GRB.EQUAL, 0, "s0-" + i); //eq3a
                expr = 0;
                expr.AddTerm(1, b[i, 0]);
                model.AddConstr(expr, GRB.EQUAL, 0, "b0-" + i);//eq3b
                expr = 0;
                expr.AddTerm(1, s[i, data.Tmax]);
                model.AddConstr(expr, GRB.EQUAL, 0, "stmax-" + i); //eq3a

                for (int t = 1; t <= data.Tmax; t++)
                {
                    GRBLinExpr c2a = 0.0;
                    c2a.AddTerm(1, s[i, t - 1]);
                    c2a.AddTerm(1, b[i, t]);
                    c2a.AddTerm(-1, s[i, t]);
                    c2a.AddTerm(-1, b[i, t - 1]);

                    for (int l = 1; l <= data.Lmax; l++)
                    {
                       c2a.AddTerm(data.D_prime[i], q[i, l, t]);
                    }

                   constr2.Add(model.AddConstr(c2a, GRB.EQUAL, data.D_it[i, t], "c2-" + i + "-" + t)); //eq2

                }
            }

            //Eq5

            for (int i = 1; i <= data.Imax; i++)
            {
                for (int l = 1; l <= data.Lmax; l++)
                {
                    for (int t = 1; t <= data.Tmax; t++)
                    {
                        var cols = columnPool.alfapool.Where(a => a.Value.machine == l && a.Value.period == t && a.Value.pattern.Sequence.Contains(i));
                        expr = 0.0;
                        foreach (var a in cols)
                        {
                            expr.AddTerm(1, alfa[a.Key]);
                        }
                        expr.AddTerm(-1, q[i, l, t]);
                        constr5.Add(model.AddConstr(expr, GRB.GREATER_EQUAL, 0, "c5-" + i + "-" + l + "-" + t));
                    }
                }
            }

            //Eq6
            for (int l = 1; l <= data.Lmax; l++)
            {
                for (int t = 1; t <= data.Tmax; t++)
                {
                    expr = 0.0;
                    for (int i = 1; i <= data.Imax; i++)
                    {
                        expr.AddTerm(data.D_prime[i] * data.CT_i[i], q[i, l, t]);
                    }
                    expr.AddTerm(1, t_plus[l, t]);
                    expr.AddTerm(1, t_minus[l, t]);

                    var alfalist = columnPool.alfapool.Where(c => c.Value.machine == l && c.Value.period == t).Select(x => x.Key).ToList();
                    foreach (var a in alfalist)
                    {
                        expr.AddTerm(columnPool.alfapool[a].pattern.ST, alfa[a]);
                    }
                    constr6.Add(model.AddConstr(expr, GRB.LESS_EQUAL, data.Tcap, "c6-" + l + "-" + t));
                }
            }


            //Eq7

            for (int l = 1; l <= data.Lmax; l++)
            {
                for (int t = 1; t <= data.Tmax; t++)
                {
                    expr = 0.0;
                    var alfalist = columnPool.alfapool.Where(c => c.Value.machine == l && c.Value.period == t).Select(x => x.Key).ToList();
                    foreach (var a in alfalist)
                    {
                        expr.AddTerm(1, alfa[a]);
                    }
                    expr.AddTerm(1, wf[l, t]);
                    constr7.Add(model.AddConstr(expr, GRB.EQUAL, 1, "c7-" + l + "-" + t));
                }
            }



            //Eq11
            for (int l = 1; l <= data.Lmax; l++)
            {
                for (int t = 1; t < data.Tmax; t++)
                {
                    var betalist = columnPool.betapool.Where(x => x.Value.col1.period == t && x.Value.col1.machine == l);
                    expr = 0.0;
                    foreach (var a in betalist)
                    {
                        expr.AddTerm(a.Value.setup, beta[a.Key]);
                    }

                    expr.AddTerm(-1, t_plus[l, t]);
                    expr.AddTerm(-1, t_minus[l, t + 1]);
                    model.AddConstr(expr, GRB.EQUAL, 0, "c10-" + l + "-" + t);
                }
            }

            

            //Eq12-13-14 - for linearization of the quadratic cost function as a constraint in the model. It adds the setup carryover time to the problem as a capacity constraint.
            foreach (var b in columnPool.betapool)
            {
                var first_column = columnPool.alfapool.Where(x => x.Value.id == b.Value.col1.id).First();
                var second_column = columnPool.alfapool.Where(x => x.Value.id == b.Value.col2.id).First();
                GRBLinExpr c11 = 0.0;
                GRBLinExpr c12 = 0.0;
                GRBLinExpr c13 = 0.0;
                c11.AddTerm(1, beta[b.Key]);
                c11.AddTerm(-1, alfa[first_column.Key]);
                constr11.Add(model.AddConstr(c11, GRB.LESS_EQUAL, 0, "c11-" + b.Key));
                c12.AddTerm(1, beta[b.Key]);
                c12.AddTerm(-1, alfa[second_column.Key]);
                constr12.Add(model.AddConstr(c12, GRB.LESS_EQUAL, 0, "c12-" + b.Key));
                c13.AddTerm(1, beta[b.Key]);
                c13.AddTerm(-1, alfa[first_column.Key]);
                c13.AddTerm(-1, alfa[second_column.Key]);
                constr13.Add(model.AddConstr(c13, GRB.GREATER_EQUAL, -1, "c13-" + b.Key));
            }

            for(int t = 1; t < data.Tmax; t++)
            {
                for(int l = 1; l <= data.Lmax; l++)
                {
                    expr = 0.0;
                    expr.AddTerm(1, t_plus[l, t]);
                    expr.AddTerm(-24*3600, stco[l, t]);
                    model.AddConstr(expr, GRB.LESS_EQUAL, 0, "ca-" + l + "-" + t);
                    expr = -24*3600;
                    expr.AddTerm(1, t_minus[l, t + 1]);
                    expr.AddTerm(24*3600, stco[l, t]);
                    model.AddConstr(expr, GRB.LESS_EQUAL, 0, "cb-" + l + "-" + t);
                }
            }

            //Fix binaries for relaxed problem
            if (isRelaxed == true)
            {
                foreach (var a in columnPool.alfapool)
                {
                    expr = 0.0;
                    expr.AddTerm(1, alfa[a.Key]);
                    model.AddConstr(expr, GRB.EQUAL, solution.alfa[a.Key], "fixalfa-" + a.Key);
                }
                foreach (var a in columnPool.betapool)
                {
                    expr = 0.0;
                    expr.AddTerm(1, beta[a.Key]);
                    model.AddConstr(expr, GRB.EQUAL, solution.beta[a.Key], "fixbeta-" + a.Key);
                }

                for (int l = 1; l <= data.Lmax; l++)
                {
                    for (int t = 1; t <= data.Tmax; t++)
                    {
                        expr = 0.0;
                        expr.AddTerm(1, wf[l, t]);
                        model.AddConstr(expr, GRB.EQUAL, solution.wf[l, t], "fixwf-" + l + "-" + t);
                        expr = 0.0;
                        expr.AddTerm(1, stco[l, t]);
                        model.AddConstr(expr, GRB.EQUAL, solution.stco[l, t], "fixstco-" + l + "-" + t);
                    }
                }
            }
        }

        public void initObj(ColumnPool columnPool, ProblemData data)
        {
            GRBLinExpr obj_lin = 0.0;
            GRBQuadExpr obj_quad = 0.0;

            for (int i = 1; i <= data.Imax; i++)
            {
                for (int t = 1; t <= data.Tmax; t++)
                {
                    obj_lin.AddTerm(data.HC_i[i], s[i, t]);
                    obj_lin.AddTerm(data.BC_i[i], b[i, t]);
                    for (int l = 1; l <= data.Lmax; l++)
                    {
                        obj_lin.AddTerm(data.PC_l[l] * data.CT_i[i] * data.D_prime[i], q[i, l, t]);
                    }
                }
            }

            foreach (var a in columnPool.alfapool)
            {
                obj_lin.AddTerm(a.Value.pattern.ST * data.SC, alfa[a.Key]);
            }
            for (int l = 1; l <= data.Lmax; l++)
            {
                for (int t = 1; t <= data.Tmax; t++)
                {
                    obj_lin.AddTerm(data.SC, t_plus[l, t]);
                    obj_lin.AddTerm(data.SC, t_minus[l, t]);
                    obj_lin.AddTerm(data.SC * 24 * 3600, wf[l, t]);
                }
            }
            
            model.SetObjective(obj_lin, GRB.MINIMIZE);
        }

        public void warmstart(ColumnPool columnPool, ProblemData data, Solution solution)
        {
            foreach (var a in columnPool.alfapool)
            {
                alfa[a.Key].Start = a.Value.start;
            }
            foreach (var b in columnPool.betapool)
            {
                beta[b.Key].Start = b.Value.start;
            }

            for (int l = 1; l <= data.Lmax; l++)
            {

                for (int t = 1; t <= data.Tmax; t++)
                {
                    int alfas = columnPool.alfapool.Where(x => x.Value.start == 1 && x.Value.machine == l && x.Value.period == t).Count();
                    wf[l, t].Start = solution.wf[l, t];
                }

            }
        }
        public void getSolutions(ColumnPool columnPool, ProblemData data, Solution solution)
        {


            //Get duals
            if (isRelaxed == true)
            {
                solution.c2 = new double[data.Imax + 1, data.Tmax + 1];
                solution.c5 = new double[data.Imax + 1, data.Lmax + 1, data.Tmax + 1];
                solution.c9 = new double[data.Imax + 1, data.Lmax + 1, data.Tmax + 1];
                solution.c6 = new double[data.Lmax + 1, data.Tmax + 1];
                solution.c7 = new double[data.Lmax + 1, data.Tmax + 1];

                foreach(var a in constr2)
                {
                    int i = int.Parse(a.ConstrName.Split("-")[1]);
                    int t = int.Parse(a.ConstrName.Split("-")[2]);
                    solution.c2[i, t] = a.Get(GRB.DoubleAttr.Pi);

                }

                foreach (var a in constr6)
                {
                    int l = int.Parse(a.ConstrName.Split("-")[1]);
                    int t = int.Parse(a.ConstrName.Split("-")[2]);
                    solution.c6[l, t] = a.Get(GRB.DoubleAttr.Pi);
                }
                foreach (var a in constr7)
                {
                    int l = int.Parse(a.ConstrName.Split("-")[1]);
                    int t = int.Parse(a.ConstrName.Split("-")[2]);
                    solution.c7[l, t] = a.Get(GRB.DoubleAttr.Pi);
                }

                foreach (var a in constr5)
                {
                    int i = int.Parse(a.ConstrName.Split("-")[1]);
                    int l = int.Parse(a.ConstrName.Split("-")[2]);
                    int t = int.Parse(a.ConstrName.Split("-")[3]);
                    solution.c5[i, l, t] = a.Get(GRB.DoubleAttr.Pi);
                }

                foreach (var a in constr9)
                {
                    int i = int.Parse(a.ConstrName.Split("-")[1]);
                    int l = int.Parse(a.ConstrName.Split("-")[2]);
                    int t = int.Parse(a.ConstrName.Split("-")[3]);
                    solution.c9[i, l, t] = a.Get(GRB.DoubleAttr.Pi);
                }
            }
            else
            {
                double productioncost = 0;
                double inventorycost = 0;
                double setupcost = 0;
                double backlogcost = 0;
                solution.productioncost = 0;
                solution.setupcost = 0;
                solution.inventorycost = 0;
                solution.backlogcost = 0;
                solution.totalcost = 0;

                //Get matrices

                solution.q_ilt = new double[data.Imax + 1, data.Lmax + 1, data.Tmax + 1];
                solution.q_it = new double[data.Imax + 1, data.Tmax + 1];
                solution.s = new double[data.Imax + 1, data.Tmax + 1];
                solution.b = new double[data.Imax + 1, data.Tmax + 1];
                solution.t_plus = new int[data.Lmax + 1, data.Tmax + 1];
                solution.t_minus = new int[data.Lmax + 1, data.Tmax + 1];
                solution.wf = new double[data.Lmax + 1, data.Tmax + 1];
                solution.alfa = new Dictionary<int, double>();
                solution.beta = new Dictionary<int, double>();

                for (int i = 1; i <= data.Imax; i++)
                {
                    for (int t = 1; t <= data.Tmax; t++)
                    {
                        double production = 0;
                        for (int l = 1; l <= data.Lmax; l++)
                        {
                            solution.q_ilt[i, l, t] = (int)Math.Round(q[i, l, t].X * data.D_prime[i]);
                            production += solution.q_ilt[i, l, t];
                            productioncost = productioncost + solution.q_ilt[i, l, t] * data.CT_i[i] * data.PC_l[l];
                        }
                        solution.q_it[i, t] = production;
                        solution.b[i, t] = (int)Math.Round(b[i, t].X);
                        solution.s[i, t] = (int)Math.Round(s[i, t].X);
                        inventorycost = inventorycost + solution.s[i, t] * data.HC_i[i];
                        backlogcost = backlogcost + solution.b[i, t] * data.BC_i[i];

                    }
                }
                for (int l = 1; l <= data.Lmax; l++)
                {
                    for (int t = 1; t <= data.Tmax; t++)
                    {
                        solution.t_plus[l, t] = (int)Math.Round(t_plus[l, t].X);
                        solution.t_minus[l, t] = (int)Math.Round(t_minus[l, t].X);
                        Console.WriteLine(t_plus[l, t].X + " " + l + " " + t);
                        solution.wf[l, t] = Math.Round(wf[l, t].X);
                        solution.stco[l, t] = stco[l, t].X;
                    }
                }
                //Get binaries
                foreach (var a in columnPool.alfapool)
                {
                    solution.alfa.Add(a.Key, Math.Round(alfa[a.Key].X));
                    setupcost = setupcost + data.SC * a.Value.pattern.ST * alfa[a.Key].X;
                }
                foreach (var b in columnPool.betapool)
                {
                    solution.beta.Add(b.Key, Math.Round(beta[b.Key].X));
                    setupcost = setupcost + data.SC * b.Value.setup * beta[b.Key].X;
                }

                for (int l = 1; l <= data.Lmax; l++)
                {
                    for (int t = 1; t <= data.Tmax; t++)
                    {
                        setupcost = setupcost + wf[l, t].X * data.SC * 3600 * 24;
                        if (wf[l, t].X > 0)
                        {
                            Console.WriteLine(l + " " + t);
                        }
                    }
                }

                foreach(var a in columnPool.alfapool)
                {
                    int l = a.Value.machine;
                    foreach(var i in a.Value.pattern.Sequence)
                    {
                        if (data.IL_il[i, l] == 0)
                        {
                            Console.WriteLine(i + " " + l);
                        }
                    }
                }

                for(int i = 1; i <= data.Imax; i++)
                {
                    for(int l = 1; l <= data.Lmax; l++)
                    {

                    }
                }

                Console.WriteLine("");
                Console.WriteLine("");

                Console.WriteLine("----------------------------SolutionStats----------------------------");
                Console.WriteLine("Production cost : " + productioncost);
                Console.WriteLine("Inventory cost : " + inventorycost);
                Console.WriteLine("Backlog cost : " + backlogcost);
                Console.WriteLine("Setup cost : " + setupcost);
                Console.WriteLine("Obj: " + model.ObjVal);

                Console.WriteLine("");
                Console.WriteLine("");
                solution.backlogcost = backlogcost;
                solution.inventorycost = inventorycost;
                solution.productioncost = productioncost;
                solution.setupcost = setupcost;
                solution.totalcost = model.ObjVal;
            }


            
            if (model.IsMIP == 1) Console.WriteLine("Gap: " + model.MIPGap);

        }

        public void solvemodel(ColumnPool columnPool, ProblemData data, Solution solution,int TL)
        {
            env = new GRBEnv();
            model = new GRBModel(env);
            model.Set(GRB.StringAttr.ModelName, "MIP_model");
            model.Parameters.OutputFlag = 1;
            initVariables(columnPool, data);
            model.Update();
            initConstraints(columnPool, data, solution);
            initObj(columnPool, data);
            warmstart(columnPool, data, solution);
            model.Parameters.TimeLimit = TL;
            model.Parameters.MIPGap = 0.001;
            model.Optimize();

            try
            {
                getSolutions(columnPool, data, solution);
                model.Dispose();
                env.Dispose();

            }
            catch (GRBException e)
            {
                solution.totalcost = 0;
                solution.setupcost = 0;
                solution.backlogcost = 0;
                solution.inventorycost = 0;
                solution.productioncost = 0;
                Console.WriteLine("Error code: " + e.ErrorCode + ". " + e.Message);
            }
        }
    }
}
