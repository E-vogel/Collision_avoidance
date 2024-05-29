clear
close all
clear gaoutfun

fun = @(x) Ob(x);

options = optimoptions('ga','Display','diagnose','PlotFcn',@gaplotbestf,'OutputFcn',@gaoutfun);
options.UseParallel = true;
options.MutationFcn = {@mutationuniform, 0.08};

options.MaxGenerations = 200; 
options.PopulationSize = 30; 

lb = [0 0 10];
ub = [25 2 25];

options.MaxStallGenerations = options.MaxGenerations;
x_ans = gpuArray(ga(fun,3,[],[],[],[],lb,ub,[],[],options));
record = gaoutfun();

function fun = Ob(x)
    fun = 0;
    for i = 1:5
        j = collision_avoidance_tracking_fun(x(1),x(2),x(3));
        fun = fun - j;
    end
end