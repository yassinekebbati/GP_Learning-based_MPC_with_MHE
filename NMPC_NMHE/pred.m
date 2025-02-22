%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Yassine Kebbati
% Date: 10/05/2022
% Control NMMPC-NMHE-Racing
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function Pr = pred(in , xmean, xstdev, ymean,ystdev, net)

        %[Zx,xxmean,xxstdev] = zscore(in) 
        Zx = (in - xmean) ./ xstdev;
        %x(1,:)=Z(1,:)*xstdev+xmean;
        
%         pr = net(Zx')
        pr = net.predict(Zx');
        %[Zy,ymean,ystdev] = zscore(x);
        Pr = pr'.*ystdev+ymean;
        
        
end