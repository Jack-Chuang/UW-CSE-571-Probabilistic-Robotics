import numpy as np

# DO NOT CHANGE THE NAME OF THIS FUNCTION
def poseEstimation( R,
                    L,
                    x_prev,
                    y_prev,
                    theta_prev,
                    delta_phi_left,
                    delta_phi_right):
    """
        Calculate the current Duckiebot pose using dead reckoning approach,
        based on the kinematic model.

        Returns:
            x_curr, y_curr, theta_curr (:double: values)
    """
    # x_curr = x_prev + R*(delta_phi_left+delta_phi_right)*np.cos(theta_prev)/2
    # y_curr = y_prev + R*(delta_phi_left+delta_phi_right)*np.sin(theta_prev)/2
    # theta_curr = theta_prev + R*(delta_phi_right-delta_phi_left)/(2*L)
        
    
    w = [R, R/L, 1]
    x = np.array(
        [
            [
                (delta_phi_left+delta_phi_right)*np.cos(theta_prev)/2,
                (delta_phi_left+delta_phi_right)*np.sin(theta_prev)/2,
                0
            ],
            [
                0,
                0,
                (delta_phi_right-delta_phi_left)/2],
            [
                x_prev,
                y_prev,
                theta_prev
            ]
        ])

    x_curr, y_curr, theta_curr = np.array(w).dot(x)
    
    return x_curr, y_curr, theta_curr
