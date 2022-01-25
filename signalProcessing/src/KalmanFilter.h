//
// Created by vis75817 on 1/11/2022.
//



#ifndef MCU2_KALMANFILTER_H
#define MCU2_KALMANFILTER_H

#include <array>

template<int M,int N, typename Numeric>
class Matrix {
    // addition operator
    friend Matrix<M,N,Numeric> operator+(const Matrix<M,N,Numeric>& lhs, const Matrix<M,N,Numeric>& rhs) {
        Matrix<M,N,Numeric> result;
        for (int i = 0; i < M; i++) {
            for (int j = 0; j < N; j++) {
                result.matrix[i][j] = lhs.matrix[i][j] + rhs.matrix[i][j];
            }
        }
        return result;
    }
    // multiplication operator
    friend Matrix<M,N,Numeric> operator*(const Matrix<M,N,Numeric>& lhs, const Matrix<M,N,Numeric>& rhs) {
        Matrix<M,N,Numeric> result;
        for (int i = 0; i < M; i++) {
            for (int j = 0; j < N; j++) {
                result.matrix[i][j] = 0;
                for (int k = 0; k < N; k++) {
                    result.matrix[i][j] += lhs.matrix[i][k] * rhs.matrix[k][j];
                }
            }
        }
        return result;
    }
private:
    std::array<std::array<Numeric,N>,M> matrix;
};

//vector class is a nx1 matrix
template<int N, typename Numeric>
using Vector = Matrix<N,1,Numeric>;

template<int N, typename Numeric>
class KalmanFilter {
public:
    KalmanFilter() {
        x = Vector<N, Numeric>(0);
        P = Matrix<N, N, Numeric>(0);
        Q = Matrix<N, N, Numeric>(0);
        R = Matrix<N, N, Numeric>(0);
        I = Matrix<N, N, Numeric>(0);
        for (int i = 0; i < N; i++) {
            I(i, i) = 1;
        }
    }
    using InputVector = std::array<Numeric, N>;
    void update(const InputVector &input);
    InputVector predict();
private:
    Vector<N, Numeric> x;
    Matrix<N, N, Numeric> P;
    Matrix<N, N, Numeric> Q;
    Matrix<N, N, Numeric> R;
    Matrix<N, N, Numeric> I;
};


#endif //MCU2_KALMANFILTER_H
