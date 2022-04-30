param N;                       # Number of Intermediate Nodes + 1;
set N1 := {0..N};              # 0 U {I} U d
set N2 := {1..N-1} ;           #  I
set N3 := {0..N-1};            # 0 U {I}
set N4 := {1..N};              # {I} U d
 
# Part1: Decision Variables
var x{N1,N1} binary;           #Select the path_{ij}
var k integer;                 #Number of buses
var u{N2};
var v{N2};
var c{N1,N1};                  #Cost_{i,j}


param d{N1,N1};				   #Distance_{i,j}
param f;                       #Fixed Cost
param Q;
param T;
param a;
 
param b{N2};                   # q_i


# Part2: Objective Function
minimize z: f*k + sum{ i in N1 , j in N1 }c[i,j]*x[i,j];

# Part 3: Constraints
s.t. M0 {i in N1, j in N1}: c[i,j] =a*d[i,j];

s.t. M1: sum{i in N2}x[0,i] = k;
s.t. M2: sum{i in N2}x[i,N] = k;
s.t. M3 { i in N2 }: sum{ j in N4 }x[i , j]  = 1;
s.t. M4 { j in N2 }: sum{ i in N3 }x[i , j]  = 1;

s.t. M5 {i in N2 , j in N2: i!=j}: u[i]-u[j]+100000*x[i,j]+(100000-b[i]-b[j])*x[j,i] <= 100000- b[j];
s.t. M6 {i in N2}: b[i]<=u[i];
s.t. M7 {i in N2}: u[i]-b[i]*x[0,i]+Q*x[0,i] <= Q ;

s.t. M8 {i in N2, j in N2: i<>j}:v[i]-v[j]+(T-d[i,N]-d[0,j]+d[i,j])*x[i,j]+(T-d[i,N]-d[0,j]-d[j,i])*x[j,i] <= T-d[i,N]-d[0,j];
s.t. M9 {i in N2}: v[i] - d[0,i]*x[0,i] >= 0;
s.t. M10 {i in N2}: v[i]-d[0,i]*x[0,i]+T*x[0,i]<=T;

#s.t. M11 {i in N2}: x[i,i]<=0;

 


 

 


 

 

 
 
 
 
 
 
 