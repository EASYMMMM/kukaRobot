% function [result,result_yci,all_intent] = GRP_xyz_onepoint(X_test, prior_intent, GPR_model,num_intent,num_GPR ,predict_length)
 function [result,result_yci,all_intent] = GRP_xyzvvv_onepoint(X_test, prior_intent, gprMdl_1,gprMdl_2,gprMdl_3, gprMdl_4, gprMdl_5, gprMdl_6 ,predict_length)
% 函数输入：Xtest是起点所有特征，比如，起点的xyz，还有三个速度。prior_intent可以和init_1的意图配合，不过影响不大。
% 然后跟6个GPR模型。最后predict_length是贝叶斯递推的长度，看着给吧，建议T_tot/Ts。 
% 函数输出：result是预测的轨迹xyz和仨速度，result_yci是std边界，all_intent是预测意图。


    all_intent=[];
    init_1=1/3;init_2=1/3;init_3=1/3; %可以改，1/意图数目，或者直接给当前时刻意图也可以。
    lambda=0.8; 
    
    num_GPR=6;num_intent=3; %可以改，顾名思义，numGPR是几个独立的GPR模型，intent数目，是多少小v的意思
    
    result=[]; result_y=[];result_y_upstd=[];result_y_downstd=[];
    nmb_p2=[];
    %%
    pt30=[X_test(1,1:end) 3];
    pt20=[X_test(1,1:end) 2];
    pt10=[X_test(1,1:end) 1];
    result_yci=[];
    result=[X_test(1,:)];

    
%%   下面的不需要改
    for t = 1:predict_length
        this_den=0;
        g_relate_pre=[];
        max_yci=[];
        all_rou=[];
        for g = 1:num_intent
            expr_g=['pt' num2str(g) '0'];
            value_g=eval(expr_g);
            expr_g_init=['init_' num2str(g)];
            init_g=eval(expr_g_init);       
            thisg_pre=[];
            each_yci=[];
            rou0=1;
            for GPR = 1:num_GPR
                expr_GPR=['gprMdl_' num2str(GPR)];
                gprMdl=eval(expr_GPR);
                [ypred_single, standard, yci] = predict(gprMdl,value_g);
                each_yci=[each_yci yci];
                expr_pre_y=['pre_y_' num2str(GPR) ' = ypred_single;' ];
                eval(expr_pre_y);
                thisg_pre=[thisg_pre ypred_single];
                rou=normpdf(ypred_single,ypred_single,standard); 
                rou0=rou0*rou;
            end
            %%
            g_relate_pre=[g_relate_pre; thisg_pre;];
            all_rou=[all_rou rou0];
            max_yci=[max_yci; each_yci;];
            this_den=this_den+rou0*init_g^lambda; % 分母
        end
        all_rou;
        all_init=[];  
        for g = 1:num_intent
            expr_g00=['init_' num2str(g)];
            init_g=eval(expr_g00);      
            rou00=all_rou(g);
            this_mem=rou00*init_g^lambda;
            next_rou_value=this_mem/this_den;
            all_init=[all_init next_rou_value];
            expr_rou=['init_' num2str(g) ' = next_rou_value;' ];
            eval(expr_rou);
        end   
% init_1
% init_2
        %比大小
         which_max=find(all_init==max(all_init));
         if length(which_max)>1
             which_max=which_max(1);
         end
         this_pre=g_relate_pre(which_max,:);

         this_yci=max_yci(which_max,:);
         for gg = 1:num_intent
                expr_newpt=['pt' num2str(gg) '0 = [this_pre gg];' ];
                eval(expr_newpt);
         end
all_intent=[all_intent;all_init;];         
         
pt10;
pt20;
nmb_p2=[nmb_p2; pt20;];
         result=[result; this_pre;];
         result_yci=[result_yci; this_yci;];
    end

nmb_p2;
end

