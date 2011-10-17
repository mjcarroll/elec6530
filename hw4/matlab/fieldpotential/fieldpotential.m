
%%
width = 20;
height = 10;
res = 0.1; % Decimeter

goal = [20,10];
katt = 5;
krep = 1;
urmax = 0.75;
po = 1000;

%%
sc = 1/res;
costmap = zeros(20*sc+2,10*sc+2);
costmap(3*sc:(3+3)*sc,3*sc:(3+3)*sc) = urmax;
costmap(9*sc:(9+3)*sc,5*sc:(5+3)*sc) = urmax;
costmap(15*sc:(15+3)*sc,4*sc:(4+3)*sc) = urmax;

%%
xgoal = goal(1)*sc; ygoal = goal(2)*sc;
for x = 1:size(costmap,1),
    for y=1:size(costmap,2),
        pgoal = (x-xgoal)^2 + (y-ygoal)^2;
        ua(x,y) = pgoal;
    end
end

uamax = max(max(ua));
ua = ua/uamax;

%%
ur = costmap;

pts = [];

res = 0.1;

pts = [pts; boxpoints(res,res,20-res,10-res,res)];
pts = [pts; boxpoints(3,3,3,3,res)];
pts = [pts; boxpoints(9,5,3,3,res)];
pts = [pts; boxpoints(15,4,3,3,res)];

ocx = pts(:,1) * sc;
ocy = pts(:,2) * sc;

for oci=1:length(ocx),
    for x=1:size(costmap,1),
        for y = 1:size(costmap,2),
            poc=sqrt((x-ocx(oci))^2+(y-ocy(oci))^2); 
                        %distance to current obstacle cell
            if (poc==0) %obstacle cells==infinity
                ur(x,y)=urmax;
            elseif (poc<po)%if within region of influence
                ur(x,y)=ur(x,y)+0.5*krep*(1/poc - 1/po)^2;
                if ur(x,y) > urmax,
                    ur(x,y) = urmax;
                end
                %calculate repulsive potential at current cell
            end
        end
    end
end

%%
usum = (katt*ua+krep*ur)/(katt+krep);
mesh(usum)
