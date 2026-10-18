===Introduction===
This repository contains the MATLAB code to implement the simulations and theoretical analysis of relative nearness-based gap detection. Details regarding the model can be found in the manuscript: 
"Xin Li, Haicong Zhou, Zhihua Wu*. "Gap detection based on relative nearness in a computational model", submitted in October 2026.

===Code=== 
* The subdirectory \simulation:
1. The program simulation_coppeliasim_main.m establishes communication between MATLAB and CoppeliaSim for the simulation control. The code was developed under CoppeliaSim Edu 4.9.0 and MATLAB R2019b. To run this script to reproduce the simulations, other four CoppeliaSim Legacy Remote API files in the subdirectory are also needed: remApi.m, remoteApiProto.m, remoteApi.dll, and remoteApiConnections.txt. 
2. Five .tif images are provided for covering the scene surfaces in CoppeliaSim. The texture can be replaced by following the steps:
  1) Open the properties of each scene object. 
  2) Navigate to "Scene Object Properties > Texture > Load new texture" and select the desired texture image.
  3) Set "Mapping mode" as "cube".
  4) Enable "Repeat along U" and "Repeat along V".
  5) Set the "Texture scaling" values "Along U" and "Along V" as 0.1.
3. The data generated from the simulations were saved as .mat files and separately stored in \fig2_data, \fig3_data, and \fig4_data according to the simulation paradigm.

* The main folder:
Each of the following programs generates a few panels of one figure in the manuscript. All the programs were developed under MATLAB R2013b, and the functions they call are provided in \sharedFunctions.  
Fig1B.m
Fig2D.m 
Fig3A.m 
Fig3B_3C.m
Fig4.m 
Fig5B1_5C_5D.m 
Fig5B2_5E_5F.m 
Fig5G1_5G2.m 

===Authors===
1. simulation_coppeliasim_main.m was written by Haicong Zhou and Xin Li.
2. Other programs that generate each of the figures were written by Xin Li and Zhihua Wu.
3. Three functions (contrast_Skelton.m, medulla_adaptation.m, and Silog.m) in the subdirectory \sharedFunctions were downloaded from https://doi.org/10.24433/CO.8721160.v2.

===License===
GPL-3.0
