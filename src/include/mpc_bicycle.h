#include <cppad/ipopt/solve.hpp>
using CppAD::AD;
class FG_eval
{
    public:
        typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
        void operator()(ADvector& fg, const ADvector X)
        {
            int N = Integer(X[X.size()-1]);
            assert(fg.size() == (2*N)+16);
            assert(X.size() == 2*N+11);

            AD<double> vel[N];
            AD<double> steering_ang[N];
            for (int i = 0; i < N; i++)
            {
                vel[i] = X[2*i];
                steering_ang[i] = X[2*i+1];
            }

            AD<double> T = X[2*N];
            AD<double> target_x = X[2*N+1];
            AD<double> target_y = X[2*N+2];
            AD<double> target_theta = X[2*N+3];
            AD<double> obs_x = X[2*N+4];
            AD<double> obs_y = X[2*N+5];
            AD<double> x_curr = X[2*N+6];
            AD<double> y_curr = X[2*N+7];
            AD<double> theta_curr = X[2*N+8];
            AD<double> L = X[2*N+9];
            AD<double> x_n = x_curr;
            AD<double> y_n = y_curr;
            AD<double> theta_n = theta_curr;
            for (int i = 0; i < N; i++)
            {
                x_n += T*vel[i]*CppAD::cos(theta_n + steering_ang[i]);
                y_n += T*vel[i]*CppAD::sin(theta_n + steering_ang[i]);
                theta_n += T*CppAD::sin(steering_ang[i])*vel[i]/L;
            }
            
            // f(x)
            // optimizing eqn:
            fg[0] = pow(x_n-target_x,2) + pow(y_n-target_y,2) + pow(theta_n-target_theta,2);

            // Constraints:

            // eqn for the distance from the obstacle is greater than 2 at each time step
            fg[1] = (x_n- obs_x)*(x_n- obs_x) + (y_n- obs_y)*(y_n- obs_y); // values greater than 4
            for (int i = 0; i < N-1; i++)
            {
                fg[2*i+2] = abs(vel[i+1] - vel[i]);
                fg[2*i+3] = abs(steering_ang[i+1] - steering_ang[i]);
            }
            fg[2*N+2] = x_n;
            fg[2*N+3] = y_n;
            fg[2*N+4] = theta_n;
            fg[2*N+5] = T;
            fg[2*N+6] = target_x;
            fg[2*N+7] = target_y;
            fg[2*N+8] = target_theta;
            fg[2*N+9] = obs_x;
            fg[2*N+10] = obs_y;
            fg[2*N+11] = x_curr;
            fg[2*N+12] = y_curr;
            fg[2*N+13] = theta_curr;
            fg[2*N+14] = N;
            fg[2*N+15] = L;
            return;
        }
};
