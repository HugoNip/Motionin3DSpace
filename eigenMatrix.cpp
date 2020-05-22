#include <iostream>
#include <ctime>
#include <Eigen/Core>
#include <Eigen/Dense>

#define MATRIX_SIZE 50

int main (int argc, char** argv)
{
    // Definition of Eigen Matrix
    Eigen::Matrix<float, 2, 3> matrix_23;

    Eigen::Vector3d v_3d; // Eigen::Matrix<double, 3, 1>
    Eigen::Matrix<float, 3, 1> vd_3d;
    Eigen::Matrix3d matrix_33 = Eigen::Matrix3d::Zero();

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> matrix_dynamic;

    Eigen::MatrixXd matrix_x;

    // operation for Eigen Matrix
    // input
    matrix_23 << 1, 2, 3, 4, 5, 6;

    // output
    std::cout << "matrix 2x3 from 1 to 6: \n" << matrix_23 << std::endl;

    // visit member
    std::cout << "print matrix 2x3: " << std::endl;
    for (int i = 0; i < 2; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            std::cout << matrix_23(i, j) << "\t";
        }
        std::cout << std::endl;
    }

    // matrix multiply
    v_3d << 3, 2, 1;
    vd_3d << 4, 5, 6;

    Eigen::Matrix<double, 2, 1> result = matrix_23.cast<double>() * v_3d;
    std::cout << "[1, 2, 3; 4, 5, 6] * [3, 2, 1] = " << result.transpose() << std::endl;

    // Matrix Operation
    matrix_33 = Eigen::Matrix3d::Random();
    std::cout << "random matrix: \n" << matrix_33 << std::endl;
    std::cout << "transpose: \n" << matrix_33.transpose() << std::endl;
    std::cout << "sum: " << matrix_33.sum() << std::endl;
    std::cout << "trace: " << matrix_33.trace() << std::endl;
    std::cout << "times 10: " << 10 * matrix_33 << std::endl;
    std::cout << "inverse: \n" << matrix_33.inverse() << std::endl;
    std::cout << "det: " << matrix_33.determinant() << std::endl;

    // Eigen Value
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(matrix_33.transpose() * matrix_33);
    std::cout << "Eigen values = \n" << eigen_solver.eigenvalues() << std::endl;
    std::cout << "Eigen vectors = \n" << eigen_solver.eigenvectors() << std::endl;

    // Solve the Equation
    // matrix_NN * x = v_Nd
    Eigen::Matrix<double, MATRIX_SIZE, MATRIX_SIZE> matrix_NN = Eigen::MatrixXd::Random(MATRIX_SIZE, MATRIX_SIZE);
    matrix_NN = matrix_NN * matrix_NN.transpose(); // positive semidefinite matrix
    Eigen::Matrix<double, MATRIX_SIZE, 1> v_Nd = Eigen::MatrixXd::Random(MATRIX_SIZE, 1);

    clock_t time_stt = clock();
    // calculate normal inverse
    Eigen::Matrix<double, MATRIX_SIZE, 1> x = matrix_NN.inverse() * v_Nd;
    std::cout << "time of normal inverse is " << 1000 * (clock() - time_stt) / (double) CLOCKS_PER_SEC << "ms" << std::endl;
    std::cout << "x = " << x.transpose() << std::endl;

    // QR decomposition
    time_stt = clock();
    x = matrix_NN.colPivHouseholderQr().solve(v_Nd);
    std::cout << "time of QR decompositon is " << 1000 * (clock() - time_stt) / (double) CLOCKS_PER_SEC << "ms" << std::endl;
    std::cout << "x = " << x.transpose() << std::endl;

    // cholesky decomposition
    time_stt = clock();
    x = matrix_NN.ldlt().solve(v_Nd);
    std::cout << "time of ldlt decomposition is " << 1000 * (clock() - time_stt) / (double) CLOCKS_PER_SEC << "ms" << std::endl;
    std::cout << "x = " << x.transpose() << std::endl;

    return 0;
}

