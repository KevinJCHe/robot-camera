#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

using namespace std;
using namespace cv;

Mat B, S, U, Vt, t_enlarged;

struct camera_extrinsic_matrices{
	Mat R;
	Mat t;
};

camera_extrinsic_matrices Find_R_and_t(Mat A, Mat B){

	camera_extrinsic_matrices Rtmatrix;
	Mat centroid_A, centroid_B, centroid_A_enlarged, centroid_B_enlarged,  AA, BB, H, s, u, vt;

	//Number of rows
	int n = A.rows;

	//Retrieve the centroids of the dataset
	reduce(A,centroid_A, 0, CV_REDUCE_AVG);
	reduce(B,centroid_B, 0, CV_REDUCE_AVG);

	//Center the points
	for(int s=0;s<n;s++){
		centroid_A_enlarged.push_back(centroid_A); 
		centroid_B_enlarged.push_back(centroid_B); 
	}
	AA = A - centroid_A_enlarged;
    BB = B - centroid_B_enlarged;

    //Find covariance matrix, decompse H using svd, calculate R
    H = AA.t() * BB;

    SVD::compute(H, s, u, vt);

	Rtmatrix.R = vt.t()*u.t();

	//special reflection case
	if(determinant(Rtmatrix.R) < 0){
	   vt.row(2) *= -1;
	   Rtmatrix.R = vt.t()*u.t();
	}

	//Find t
    Rtmatrix.t = - Rtmatrix.R * centroid_A.t() + centroid_B.t();

    return Rtmatrix;
}

int main(){

	//ALL THIS IS JUST SETTING UP THE RANDOM DATASET POINTS
	/*
	A = random dataset 1
	B = randomly rotated and translated dataset 1
	*/

	//recover the transformation
	camera_extrinsic_matrices R_t_matrices;
	R_t_matrices = Find_R_and_t(A, B);
	
	cout << format(R_t_matrices.R,"python") << endl << endl;
	cout << R_t_matrices.t << endl << endl;

}