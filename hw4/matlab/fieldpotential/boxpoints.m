function [ pts ] = boxpoints( x,y,w,h,res)

pts = [];
for i=colon(x,res,x+w),
   pts = [pts;
       i,y;
       i,y+h]; 
end

for j=colon(y,res,y+h)
    pts = [pts;
        x,j;
        x+w,j];
end

end

