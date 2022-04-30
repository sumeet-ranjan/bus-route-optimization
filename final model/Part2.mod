#Part1 Parameters
param r0;
set R = {1..r0};

param n;
set N = {1..n};
set S := 1 .. n;
set SS := 1 .. 2**n;
set POW {k in SS} := {i in S: (k div 2**(i-1)) mod 2 = 1};

param cap;
param D{N,N};
param e{R};
param f{R};
param l{N,N};                 #C_bus
param Ca{N,N};				  #C_alt
param trmax;
param M;

# Part2: Decision Variables
var x{N,N,R} binary;          #[I][J][R]
var p{N,N,N,N,R} binary;      #[O][D][I][J][R]
var t{N,N,N} binary;          #[O][D][I]
var m{N,N} binary;
var Cb{N,N};
var u{N,N,R};

# Part3: Objective Function
maximize z: sum {o in N, d in N} (D[o,d]*m[o,d]) -sum{o in N, d in N}Cb[o,d] -sum{i in N, j in N, r in R} (l[i,j]*x[i,j,r])-  0.001*sum{o in N, d in N, i in N} (m[o,d]*D[o,d]*t[o,d,i]);

# Part 4: Constraints


### route start and end###
s.t. M01 {r in R}: sum{j in N} x[e[r],j,r]=1;
s.t. M02 {r in R}: sum{j in N} x[j,f[r],r]=1;

### flow conservation ###
s.t. M03 {i in N, r in R: i!=e[r] and i!=f[r]}: sum{j in N}x[j,i,r]=sum{k in N}x[i,k,r];

### distance travelled by the bus ###
s.t. N01{o in N, d in N}: Cb[o,d]=sum{i in N, j in N, r in R}l[i,j]*p[o,d,i,j,r];

#s.t. M14 {i in N, r in R}: x[i,i,r] = 0;

### sub tour elimination ###
s.t. M04 {s in SS, r in R:card(POW[s])>=2 }:sum{i in POW[s], j in POW[s]: i<>j}x[i,j,r]<=card(POW[s])-1;

#s.t. N01 {r in R, i in N , j in N : i<j}: x[j,i,r] = x[i,j,r];


### flow conservation of group ###
s.t. M05 {o in N, d in N}: sum{j in N, r in R}p[o,d,o,j,r] = 1;
s.t. M06 {o in N, d in N}: sum{j in N, r in R}p[o,d,j,d,r] = 1;  
s.t. M07 {o in N, d in N, i in N: i!=o and i!=d}: sum{j in N, r in R:i<>j}p[o,d,j,i,r] = sum{k in N, r in R:i<>k}p[o,d,i,k,r];

#s.t. M08 {o in N, d in N}: Cb[o,d]<=Ca[o,d]+M*(1-m[o,d]);

### bus transfer ###
s.t. M09 {o in N, d in N, r in R, i in N: i!=o and i!=d}: sum{j in N}p[o,d,j,i,r] - sum{k in N}p[o,d,i,k,r]<=t[o,d,i];
#s.t. M10 {o in N, d in N}: sum{i in N} t[o,d,i] <= trmax;

### group travel along bus ###
s.t. M11 {r in R , i in N, j in N}: sum{o in N, d in N} (p[o,d,i,j,r]+p[o,d,j,i,r]) <= M*(x[i,j,r]+x[j,i,r]);


### capacity constraint ###
s.t. M68 {i in N,j in N, r in R}: (sum{o in N,d in N}D[o,d]*p[o,d,i,j,r]*m[o,d])<=cap;
s.t. M69 {i in N,j in N, r in R}: u[i,j,r] <= (sum{o in N,d in N}D[o,d]*p[o,d,i,j,r]*m[o,d]);
s.t. M70 {i in N,j in N, r in R}: u[i,j,r] >= (sum{o in N,d in N}D[o,d]*p[o,d,i,j,r]*m[o,d]);
#s.t. M69 {i in N, r in R}:u[i,r]<=cap;

### bus switch constraint ###
#s.t. bus {o in N, d in N}: sum{i in N}t[o,d,i]<=0+M*(1-m[o,d]); 







