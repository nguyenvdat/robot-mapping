function [v] = t2v(t)
	v = [t(1,3); t(2,3); atan2(t(2,1), t(1,1))];
end