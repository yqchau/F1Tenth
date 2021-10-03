#include "scan_matching_skeleton/transform.h"
#include <cmath>
#include "ros/ros.h"
#include <Eigen/Geometry>
#include <complex>

using namespace std;


void transformPoints(const vector<Point>& points, Transform& t, vector<Point>& transformed_points) {
  transformed_points.clear();
  for (int i = 0; i < points.size(); i++) {
    transformed_points.push_back(t.apply(points[i]));
    //printf("%f %transformed_points.back().r, transformed_points.back().theta);
  }
}

// returns the largest real root to ax^3 + bx^2 + cx + d = 0
complex<float> get_cubic_root(float a, float b, float c, float d) {
  //std::cout<< "a= " << a<< ";  b= " << b<< ";  c= " << c<< ";  d= " << d<<";"<<std::endl;
  // Reduce to depressed cubic
  float p = c/a - b*b/(3*a*a);
  float q = 2*b*b*b/(27*a*a*a) + d/a - b*c/(3*a*a);

  // std::cout<<"p = "<<p<<";"<<std::endl;
  // std::cout<<"q = "<<q<<";"<<std::endl;

  complex<float> xi(-.5, sqrt(3)/2);

  complex<float> inside = sqrt(q*q/4 + p*p*p/27);

  complex<float> root;

  for (float k = 0; k < 3; ++k) {
    // get root for 3 possible values of k
    root = -b/(3*a) + pow(xi, k) * pow(-q/2.f + inside, 1.f/3.f) + pow(xi, 2.f*k) * pow(-q/2.f - inside, 1.f/3.f);
    //std::cout<<"RootTemp: "<< root<<std::endl;
    if (root.imag() != 0) { return root; }
  }

  return root;
}

// returns the largest real root to ax^4 + bx^3 + cx^2 + dx + e = 0
float greatest_real_root(float a, float b, float c, float d, float e) {
  // Written with inspiration from: https://en.wikipedia.org/wiki/Quartic_function#General_formula_for_roots
  //std::cout<< "a= " << a<< ";  b= " << b<< ";  c= " << c<< ";  d= " << d<< ";  e= " << e<<";"<<std::endl;

  // Reduce to depressed Quadratic
  float p = (8*a*c-3*b*b)/(8*a*a);
  float q = (b*b*b-4*a*b*c+8*a*a*d)/(8*a*a*a);
  float r = (-3*b*b*b*b+256*a*a*a*e-64*a*a*b*d+16*a*b*b*c)/(256*a*a*a*a);

  // std::cout<<"p = "<<p<<";"<<std::endl;
  // std::cout<<"q = "<<q<<";"<<std::endl;
  // std::cout<<"r = "<<r<<";"<<std::endl;

  // Ferrari's Solution for Quartics: 8m^3 + 8pm^2 + (2p^2-8r)m - q^2 = 0
  complex<float> m = get_cubic_root(8, 8*p, 2*p*p-8*r, -q*q);

  complex<float> root1 = -b/(4*a) + ( sqrt(2.f*m) + sqrt(-(2*p + 2.f*m + sqrt(2.f)*q/sqrt(m))))/2.f;
  complex<float> root2 = -b/(4*a) + ( sqrt(2.f*m) - sqrt(-(2*p + 2.f*m + sqrt(2.f)*q/sqrt(m))))/2.f;
  complex<float> root3 = -b/(4*a) + (-sqrt(2.f*m) + sqrt(-(2*p + 2.f*m - sqrt(2.f)*q/sqrt(m))))/2.f;
  complex<float> root4 = -b/(4*a) + (-sqrt(2.f*m) - sqrt(-(2*p + 2.f*m - sqrt(2.f)*q/sqrt(m))))/2.f;

  vector<complex<float>> roots { root1, root2, root3, root4 };

  float max_real_root = 0.f;

  for (complex<float> root: roots) {
    if(root.imag()==0){
    max_real_root = max(max_real_root, root.real());
    }
  //std::cout<<"Max real root:" << max_real_root<<std::endl;
  }
  return max_real_root;
}

void updateTransform(vector<Correspondence>& corresponds, Transform& curr_trans) {
  // Written with inspiration from: https://github.com/AndreaCensi/gpc/blob/master/c/gpc.c
  // You can use the helper functions which are defined above for finding roots and transforming points as and when needed.
  // use helper functions and structs in transform.h and correspond.h
  // input : corresponds : a struct vector of Correspondene struct object defined in correspond.
  // input : curr_trans : A Transform object refernece
  // output : update the curr_trans object. Being a call by reference function, Any changes you make to curr_trans will be reflected in the calling function in the scan_match.cpp program/

  // You can change the number of iterations here. More the number of iterations, slower will be the convergence but more accurate will be the results. You need to find the right balance.
  int number_iter = 1000;

  // iterate until converged
  for(int i = 0; i<number_iter; i++){

    //fill in the values of the matrics
    Eigen::MatrixXf M_i(2, 4);
    Eigen::MatrixXf n_i(2, 1);
    Eigen::Matrix2f C_i;
    Eigen::Vector2f pi_i;

    // Fill in the values for the matrices
    Eigen::Matrix4f M, W;
    Eigen::MatrixXf g(4, 1);
    M << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    W << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
    g << 0, 0, 0, 0;


    for (i=0; i<corresponds.size(); i++){
      M_i << 1, 0, corresponds[i].p->getX(), -corresponds[i].p->getY(), 0, 1, corresponds[i].p->getY(), corresponds[i].p->getX();
      n_i = corresponds[i].getNormalNorm();
      pi_i << corresponds[i].pj1->getX(), corresponds[i].pj1->getY();
      C_i = n_i*n_i.transpose();

      M += M_i.transpose()*C_i*M_i;
      g += (-2*pi_i.transpose()*C_i*M_i).transpose();
    }

    // Define sub-matrices A, B, D from M
    Eigen::Matrix2f A, B, D;
    //define S and S_A matrices from the matrices A B and D
    Eigen::Matrix2f S;
    Eigen::Matrix2f S_A;

    A << 2*M.block(0,0,2,2);
    B << 2*M.block(0,2,2,2);
    D << 2*M.block(2,2,2,2);

    S << D-(B.transpose())*(A.inverse())*B;
    S_A << (S.determinant())*(S.inverse());

    float f_1, f_2, f_3;
    Eigen::Matrix4f F_1, F_2, F_3;

    F_1.setZero(4,4);
    F_2.setZero(4,4);
    F_3.setZero(4,4);
    F_1.topLeftCorner(2,2) = A.inverse()*B*B.transpose()*A.inverse().transpose();
    F_1.bottomLeftCorner(2,2) = -(A.inverse()*B).transpose();
    F_1.topRightCorner(2,2) = -A.inverse()*B;
    F_1.bottomRightCorner(2,2).setIdentity();
    //cout << F_1;
    F_2.topLeftCorner(2,2) = A.inverse()*B*S_A*B.transpose()*A.inverse().transpose();
    F_2.bottomLeftCorner(2,2) = -(A.inverse()*B*S_A).transpose();
    F_2.topRightCorner(2,2) = -A.inverse()*B*S_A;
    F_2.bottomRightCorner(2,2) = S_A;
    //cout << F_2;
    F_3.topLeftCorner(2,2) = A.inverse()*B*S_A.transpose()*S_A*B.transpose()*A.inverse().transpose();
    F_3.bottomLeftCorner(2,2) = -(A.inverse()*B*S_A.transpose()*S_A).transpose();
    F_3.topRightCorner(2,2) = -A.inverse()*B*S_A.transpose()*S_A;
    F_3.bottomRightCorner(2,2) = S_A.transpose()*S_A;
    
    //cout << g;
    f_1 = (4*g.transpose()*F_1*g)(0);
    f_2 = (4*g.transpose()*F_2*g)(0);
    f_3 = (g.transpose()*F_3*g)(0);

    float a_1, b_1, c_1, d_1;
    a_1 = S(0,0);
    b_1 = S(0,1);
    c_1 = S(1,0);
    d_1 = S(1,1);
    
    float A_1,B_1,C_1,D_1,E_1;
    A_1 = 16;
    B_1 = 16*(a_1+d_1);
    C_1 = 16*a_1*d_1 + 4*a_1*a_1 + 4*d_1*d_1 - 8*c_1*b_1 - f_1;
    D_1 = -4*a_1*b_1*c_1 - 4*b_1*c_1*d_1 + 4*a_1*a_1*d_1 + 4*a_1*d_1*d_1 -f_2;
    E_1 = a_1*a_1*d_1*d_1 - 2*a_1*b_1*c_1*d_1 + b_1*b_1*c_1*c_1 - f_3;

    //find the coefficients of the quadratic function of lambda
    // find the value of lambda by solving the equation formed. You can use the greatest real root function
    float lambda = greatest_real_root(A_1,B_1,C_1,D_1,E_1);
    // float lambda = greatest_real_root(0, 0, f_1, f_2, f_3);

    //find the value of x which is the vector for translation and rotation
    Eigen::MatrixXf x(1,4);
    x = -((2*M + 2*lambda*W).inverse()).transpose()*g;
    float theta = atan2(x(3), x(2));
    curr_trans= Transform(x(0), x(1), theta);
  }
}
