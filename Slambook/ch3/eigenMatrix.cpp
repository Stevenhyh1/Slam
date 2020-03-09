#include<iostream>
#include<ctime>

#include<Eigen/Core>
#include<Eigen/Dense>

using namespace std;

#define MATRIX_SIZE 50

int main(int argc, char *argv[]) {
    
    //Declarations
    Eigen::Matrix <float, 2, 3> matrix_23;
    Eigen::Vector3d v_3d;
    Eigen::Matrix3d matrix_33 = Eigen::Matrix3d::Zero();
    Eigen::Matrix <double, Eigen::Dynamic, Eigen::Dynamic> matrix_dynamic;
    Eigen::MatrixXd matrix_xd;

    //Input and Output
    matrix_23 << 1, 2, 3, 4, 5, 6;
    cout << matrix_23 << endl;

    for (int i=0; i<1; i++) {
        for (int j=0; j<2; j++) {
            cout << matrix_23(i,j) << endl;
        }
    }

    v_3d << 3, 2, 1;


    //Matrix Calculations
    Eigen::Matrix<double, 2, 1> result = matrix_23.cast<double>() * v_3d;
    cout << result << endl;
    matrix_33 = Eigen::Matrix3d::Random();
    cout << matrix_33 << endl;

    cout << matrix_33.transpose() << endl;
    cout << matrix_33.sum() << endl;
    cout << matrix_33.trace() << endl;
    cout << 10 * matrix_33 << endl;
    cout << matrix_33.inverse() << endl;
    cout << matrix_33.determinant() << endl;

    //Eigenvalues
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver (matrix_33*matrix_33.transpose());
    cout << "Eigen values: " << eigen_solver.eigenvalues() << endl;
    cout << "Eigen vectors: " << eigen_solver.eigenvectors() << endl;

    //Solve Linear Equations
    // Ax = b
    Eigen::Matrix <double, MATRIX_SIZE, MATRIX_SIZE> matrix_NN;
    matrix_NN = Eigen::MatrixXd::Random(MATRIX_SIZE, MATRIX_SIZE);
    Eigen::Matrix <double, MATRIX_SIZE, 1> v_Nd;
    v_Nd = Eigen::VectorXd::Random(MATRIX_SIZE);

    clock_t clock_stt = clock();
    Eigen::Matrix <double, MATRIX_SIZE,1> x = matrix_NN.inverse()*v_Nd;
    clock_t clock_end = clock();
    cout << "Time used in normal inverse is: " << 1000 * (clock_end-clock_stt)/(double)CLOCKS_PER_SEC << "ms" << endl;

    clock_stt = clock();
    x = matrix_NN.colPivHouseholderQr().solve(v_Nd);
    clock_end = clock();
    cout << "Time used in QR decomposition is: " << 1000 * (clock_end-clock_stt)/(double)CLOCKS_PER_SEC << "ms" << endl;

    

    return 0;
}
