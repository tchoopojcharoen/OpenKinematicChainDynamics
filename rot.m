function R = rot(angle,ax)
if lower(ax)=='x'
    R = [   1   0           0           ;
            0   cos(angle)  -sin(angle) ;
            0   sin(angle)  cos(angle)  ];
elseif lower(ax)=='y'
    
    R = [   cos(angle)  0   sin(angle)  ;
            0           1   0           ;
            -sin(angle) 0   cos(angle)  ];
elseif lower(ax)=='z'
    R = [   cos(angle)  -sin(angle) 0   ;
            sin(angle)  cos(angle)  0   ;
            0           0           1   ];
else
    error('invalid axis')
end
end