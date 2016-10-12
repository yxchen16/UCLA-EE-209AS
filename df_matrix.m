function df_transformation_rad = df_matrix(a,alpha,d,theta)
c_t = cos(theta);
c_a = cos(alpha);
s_t = sin(theta);
s_a = sin(alpha);

df_transformation_rad = [c_t,     -s_t,    0,       a;
                         s_t*c_a, c_t*c_a, -s_a,    -d*s_a;
                         s_t*s_a, c_t*s_a, c_a,     d*c_a;
                         0,       0,       0,       1];
end