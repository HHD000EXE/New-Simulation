function [edge,slip]=ObstacleShow(LogNum,LogDiameter,LogSpacing)
%obstacle distribution (evenly-spaced)
clear edge;
% log positions
edge = zeros(1,LogNum*3);
slip = zeros(1,LogNum*3);
for ii=0:LogNum%/2
    %edge (1+ii*3) = (LogDiameter+LogSpacing)*ii+ii*(ii+1)/2*0.1*LogSpacing;                  %ii*(ii+1)/2*0.2*LogSpacing is for unevenly spaced obstacle % gradually changed spacing
    %edge (2+ii*3) = LogDiameter/2 + (LogDiameter+LogSpacing)*ii+ii*(ii+1)/2*0.1*LogSpacing;  %ii*(ii+1)/2*0.2*LogSpacing is for unevenly spaced obstacle % gradually changed spacing
    %edge (3+ii*3) = LogDiameter + (LogDiameter+LogSpacing)*ii+ii*(ii+1)/2*0.1*LogSpacing;    %ii*(ii+1)/2*0.2*LogSpacing is for unevenly spaced obstacle % gradually changed spacing
    edge (1+ii*3) = (LogDiameter+LogSpacing)*ii;
    edge (2+ii*3) = LogDiameter/2 + (LogDiameter+LogSpacing)*ii;
    edge (3+ii*3) = LogDiameter + (LogDiameter+LogSpacing)*ii;
    % slip direction in log field
    for jj=1:length(edge)
       if mod(jj,3)==0 %flat ground
           slip(jj) = 0; %no slip
       elseif mod(jj,3)==1 % near half
           slip(jj) = -1; %slip back (to nearest edge)
       elseif mod(jj,3)==2 % far half
           slip(jj) = 1; %slip forward (to nearest edge)
       end
    end
end

% for ii=LogNum/2:LogNum
%     %edge (1+ii*3) = (LogDiameter+LogSpacing)*ii+ii*(ii+1)/2*0.1*LogSpacing;                  %ii*(ii+1)/2*0.2*LogSpacing is for unevenly spaced obstacle % gradually changed spacing
%     %edge (2+ii*3) = LogDiameter/2 + (LogDiameter+LogSpacing)*ii+ii*(ii+1)/2*0.1*LogSpacing;  %ii*(ii+1)/2*0.2*LogSpacing is for unevenly spaced obstacle % gradually changed spacing
%     %edge (3+ii*3) = LogDiameter + (LogDiameter+LogSpacing)*ii+ii*(ii+1)/2*0.1*LogSpacing;    %ii*(ii+1)/2*0.2*LogSpacing is for unevenly spaced obstacle % gradually changed spacing
%     edge (1+ii*3) = (LogDiameter+LogSpacing+0.4)*ii-0.4*LogNum/2;
%     edge (2+ii*3) = LogDiameter/2 + (LogDiameter+LogSpacing+0.4)*ii-0.4*LogNum/2;
%     edge (3+ii*3) = LogDiameter + (LogDiameter+LogSpacing+0.4)*ii-0.4*LogNum/2;
%     
%     % slip direction in log field
%     for jj=1:length(edge)
%         if mod(jj,3)==0 %flat ground
%             slip(jj) = 0; %no slip
%         elseif mod(jj,3)==1 % near half
%             slip(jj) = -1; %slip back (to nearest edge)
%         elseif mod(jj,3)==2 % far half
%             slip(jj) = 1; %slip forward (to nearest edge)
%         end
%     end
% end

end
