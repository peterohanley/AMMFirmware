hold on
%plot(hat(:,1),hat(:,2))

t = [0.2737, 0.289, 0.3148, 0.344, 0.3614];
v = [0.3396, 0.09, 1.9, -0.1, 0.3835];

%.9 too high
%.8 too high
%.7 too high
idlelevel = 0.6;
v1diff = v(1) - v(2);
v5diff = v(5) - v(2);
v = v + .9;
v(3) = v(3)-.5;
v([2 4]) = idlelevel;
v(3) = 5;%5;%2.3;
v(1) = v(2) + v1diff;
v(5) = v(2) + v5diff;
plot(t,v)

len = t(end) - t(1)
(t-t(1))
(v + 2.3-max(v))

find(t>0.3,1,'first')

bucket_hi = @(x) find(t>x,1,'first');
bucket_lo = @(x) find(t<x,1,'last');
v_lo = @(x) v(bucket_lo(x));
v_hi = @(x) v(bucket_hi(x));
t_lo = @(x) t(bucket_lo(x));
t_hi = @(x) t(bucket_hi(x));

%val @ x = vlo + vhi * (x-lo)/(hi-lo) 

val = @ (x) v_lo(x) + (v_hi(x)-v_lo(x)) * (x - t_lo(x))/(t_hi(x)-t_lo(x));

ts = linspace(t(1)+0.001,t(end)-0.001,100);
vs = zeros(size(ts));

for ix = 1:100
    vs(ix) = val(ts(ix));
end

plot(ts,vs,'x')
ylim([0,5])
% todo: choose number of points, choose height, generate value

%% generate value
ms = 68;

ts = linspace(t(1)+0.001,t(end)-0.001,ms);
vs = zeros(size(ts));

for ix = 1:ms
    vs(ix) = val(ts(ix));
end
vs = vs / 5.0 * 255;
s = '{';
for ix = 1:(ms-1)
    s = [s '0x' sprintf('%x',round(vs(ix))) ','];
end
s = [s '0x' sprintf('%x',round(vs(ms))) '}'];
k = idlelevel * 255.0/5
