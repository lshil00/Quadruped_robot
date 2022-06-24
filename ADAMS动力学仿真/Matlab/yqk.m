Function [sys,x0,str,ts]=yqk(t,x,u,flag)
switch flag
    case 0
        [sys,x0,str,ts]=mdlInitializeSizes;
    case 1
        sys=mdlDerivatives(t,x,u);
    case 2
        sys=mdlUpdate(t,x,u);
    case 3
        sys=mdlOutputs(t,x,u);
    case 4
        sys=mdlGetTimeOfNextVarHit(t,x,u);
    case 9
        sys=mdlTerminate(t,x,u);
          error(['Unhandled flag =',num2str(flag)]);
 end
 function [sys,x0,str,ts]=mdlInitializeSizes
sizes=simsizes;
sizes.NumContStates=1;
sizes.NumDiscStates=0;
sizes.NumOutputs=1;
sizes.NumInputs=1;
sizes.DirFeedthrough=1;
sizes.NumSampleTimes=1;
 sys=simsizes(sizes);
 ts=[0 0];
 end
 
 function sys=mdlDerivatives(t,x,u)
     sys=1;
 end
 
 function sys=mdlUpdate(t,x,u)
     sys=[];
 end
 
 function  sys=mdlOutputs(t,x,u)  
     sys=5.5*sin(2*pi/0.4*u+pi);
 end
 
function sys=mdlGetTimeOfNextVarHit(t,x,u)
    sampleTime=1;
    sys=t+sampleTime;
end

 function sys=mdlTerminate(t,x,u)
        sys=[];
 end
 