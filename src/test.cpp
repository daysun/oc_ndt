#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
using namespace Eigen;
using namespace std;

void Eig()
{
    Matrix2d A,B;
    A << 1, -1,1,1;
    B<<2,0,0,4;
    cout<<A*B*A.inverse()<<endl;
//    A << 1, 2, 3, 4, 5, 6, 7, 8, 9;
//    cout << "Here is a 3x3 matrix, A:" << endl << A << endl << endl;
//    EigenSolver<Matrix3d> es(A);
//    Matrix3d D = es.pseudoEigenvalueMatrix();
//    Matrix3d V = es.pseudoEigenvectors();
//    cout << "The pseudo-eigenvalue matrix D is:" << endl << D << endl;
//    cout << "The pseudo-eigenvector matrix V is:" << endl << V << endl;
//    cout << "Finally, V * D * V^(-1) = " << endl << V * D * V.inverse() << endl;
}
int main()
{

    Eig();
    return 0;

}
