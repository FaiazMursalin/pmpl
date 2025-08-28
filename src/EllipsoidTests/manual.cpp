// Program to generate the graph for cubby
#include <iostream>
#include <fstream>
#include <string>
#include <vector>

using namespace std;

int main()
{
 
 vector<double> x({10.5, 14.5, 18.5}), 
  y({9.8, 7.2, 4.6, 0, -4.6, -7.2, -9.8}),
  z({19.6, 17.0, 14.4, 11.8, 9.2, 6.6, 4.0, 1.4, -1.2});
  
 int num_vertices = x.size()*y.size()*z.size();
 int num_edges = ((x.size()-1)*y.size()*z.size()) + ((y.size()-1)*z.size()*x.size()) + ((z.size()-1)*x.size()*y.size());
 
 cout<<num_vertices<<" "<<num_edges<<endl;
 int v = 1;
 for(unsigned int i = 0; i < x.size(); i++)
   for(unsigned int j = 0; j<y.size(); j++)
     for(unsigned int k= 0; k < z.size(); k++)
       cout<<v++<<" "<<x[i]<<" "<<y[j]<<" "<<z[k]<<endl;
       
 for(int i = 1; i <= num_vertices; i++) {
     int xi = (i-1)/(y.size()*z.size());
     int yi = ((i-1)/z.size())%y.size();
     int zi = (i-1)%z.size();
     
     int nz = i+1;
     int ny = i+z.size();
     int nx = i+(y.size()*z.size());
     
     if(i % z.size() != 0 && nz <= num_vertices){
       int xj = (nz-1)/(y.size()*z.size());
       int yj = ((nz-1)/z.size())%y.size();
       int zj = (nz-1)%z.size();
       cout<<i <<" "<<nz<<" 2 "<<x[xi]<<" "<<y[yi]<<" "<<z[zi]<<" "<< x[xj]<<" "<<y[yj]<<" "<<z[zj]<<endl;
     }
     
     if(((i-1)/z.size() + 1)%y.size() != 0 && ny <= num_vertices){
       int xj = (ny-1)/(y.size()*z.size());
       int yj = ((ny-1)/z.size())%y.size();
       int zj = (ny-1)%z.size();
       cout<<i <<" "<<ny<<" 2 "<<x[xi]<<" "<<y[yi]<<" "<<z[zi]<<" "<< x[xj]<<" "<<y[yj]<<" "<<z[zj]<<endl;
     }
     
     if(nx <= num_vertices){
       int xj = (nx-1)/(y.size()*z.size());
       int yj = ((nx-1)/z.size())%y.size();
       int zj = (nx-1)%z.size();
       cout<<i <<" "<<nx<<" 2 "<<x[xi]<<" "<<y[yi]<<" "<<z[zi]<<" "<< x[xj]<<" "<<y[yj]<<" "<<z[zj]<<endl;
     }
 }
 
}

