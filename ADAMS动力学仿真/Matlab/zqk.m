function [sys,x0,str,ts]=zqk(t,x,u,flag)
switch flag
    case 0
        [sys,x0,str,ts]=mdlInitializeSizes;
    case 1
        sys=mdlDerivatives;
    case 2
        sys=mdlUpdate;
    case 3
        sys=mdlOutputs(t,x,u);
    case 4
        sys=mdlGetTimeOfNextVarHit(t,x,u);
    case 9
        sys=mdlTerminate;
          error(['Unhandled flag =',num2str(flag)]);
end
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
 
 function sys=mdlDerivatives
     sys=1;
 end
 
 function sys=mdlUpdate
     sys=[];
 end
 
 function  sys=mdlOutputs(~,~,u)  
     sys=5.5*sin(2*pi/0.4*u);
 end
 
function sys=mdlGetTimeOfNextVarHit(t,~,~)
    sampleTime=1;
    sys=t+sampleTime;
end

 function sys=mdlTerminate
        sys=[];
 end
 