function average=mean_sp(arrary)
average=0;temp=0;num=0;
for tt=1:length(arrary)
    if arrary(tt) < 120
        temp = temp + arrary(tt);
        num=num+1;
        average = temp/num;
    else
        continue;
    end
end
end
