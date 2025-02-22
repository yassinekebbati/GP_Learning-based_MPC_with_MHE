# GP_Learning-based_MPC_with_MHE


This is the implementation of the work published in the following paper "Learning-based model predictive control with moving horizon state estimation for autonomous racing".
The paper is freely accessible at this link: [https://hal.science/hal-03485108v1/preview/ICCMA2021_YK_VP_NA_VV_DI_Optimized_adaptive_MPC_for_lateral_control_of.pdf ](https://univ-evry.hal.science/hal-04745064/) It is also available at International Journal of Control : https://www.tandfonline.com/doi/abs/10.1080/00207179.2024.2409305

## Steps to run the code:

This implementation requires MATLAB, CASADI and at least one solver such as gurobi, quadprog...etc (all necessary files are included with the code).

-  # Multishooting NMPC-NMHE: 
   ### This part is the implementation of nonlinear model predictive control (NMPC) with nonlinear moving horizon state estimation (NMHE) for autonomous racing
     1. Run the script 'NMPC_NMHE_multishooting.m'.
     2. The script loads trajectory data (L-shaped/Oval racing track), lines 29-59 load the Normal oval track, lines 63-93 load the L-shaped track, and lines 98-127 load the special oval track.
     3. The script loads the GP corrections for the vehicle model based on the type of track, you can find this in lines 58 and 91.
     4. The script will also define the vehicle and controller parameters, which are used within the CASADI framework to create controller and estimator objects.
     5. Simulation starts with the chosen parameters. You can try different trajectories and change the MPC parameters to compare its performance.
     6. Note that it is important to adjust MPC constraints accordingly, or it will diverge.


 -  # Running NMPC-NMHE with GP corrections: 

     1. For the L-shape track uncomment lines 98-129 to load track data and lines 405-408 to apply corrections learned by Gaussian process regression (learned mismatch between MPC predictions and ground truth from the vehicle model).
     2. For special Oavl track uncomment lines 63-91 to load L-shaped track data and lines 405-408 to apply corrections learned by Gaussian process regression (learned mismatch between MPC predictions and ground truth from the vehicle model).
     3. To run the controller/estimator for the normal oval track without any corrections, uncomment lines 29-59 in this case lines 405-408 will apply zero correction.
     

-  # Running NMPC-NMHE with RNN neural network corrections: 
  ### To apply RNN predictions to correct the mismatch between MPC predictions and ground truth from the vehicle model do the following steps and run the script.

     1. Comment out the part loading GP corrections data in lines 58,91 or 128 depending on the track, and comment out the GP correction part in lines 405-408.
     3. Uncomment lines 134-137 to load the trained neural networks for the different errors to be corrected.
     4. Uncomment lines 389-400 to apply neural network predictions and make model mismatch corrections. 

- # Training the Gaussian process regressors for MPC model mismatch corrections
  comming soon ..

- # Building and training an MLP neural network to predict MPC model mismatch
### For this part you need to have Python with the following libraries installed
![libraries](https://github.com/user-attachments/assets/7db48209-08e3-4ea0-a23a-60a4daf6e053)


1. Run python script MLP_NMPC.py and it will build a simple neural network and train it to predict model mismatch for one parameter such as lateral position for instance, once finished the trained neural network will be saved to '.h5' file.
2. You can modify the script to build and train other neural networks for the other parameters such as longitudinal position X or vehicle orientation PSI...etc.
3. Once the neural network is available you net import it to matlab using the code in import_tf_network.m.
4. You can adjust the control algorithm provided in folder NMPC_NMHE to use the new generated neural network.
  
       
### You might want to check a closely related implementation in this repository (https://github.com/yassinekebbati/GA-optimized-MLP-based-LPV_MPC)

### If you find this work useful or use it in your work, please cite the main paper:
#### APA:
Kebbati, Y., Rauh, A., Ait-Oufroukh, N., Ichalal, D., & Vigneron, V. (2024). Learning-based model predictive control with moving horizon state estimation for autonomous racing. International Journal of Control, 1-11.

#### BibTeX:
@article{kebbati2024learning,
  title={Learning-based model predictive control with moving horizon state estimation for autonomous racing},
  author={Kebbati, Yassine and Rauh, Andreas and Ait-Oufroukh, Naima and Ichalal, Dalil and Vigneron, Vincent},
  journal={International Journal of Control},
  pages={1--11},
  year={2024},
  publisher={Taylor \& Francis}
}

