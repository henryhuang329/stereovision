#include <iostream>
#include <stdio.h>
#include <math.h>

#include "io_disp.h"
#include "utils.h"
#include "stereoVision.h"

using namespace std;
const int MAX_ERROR = 60;

vector<float> disparityErrorsOutlier (DisparityImage &D_gt,DisparityImage &D_orig,DisparityImage &D_ipol) {

  // check file size
  if (D_gt.width()!=D_orig.width() || D_gt.height()!=D_orig.height()) {
    cout << "ERROR: Wrong file size!" << endl;
    throw 1;
  }

  // extract width and height
  int32_t width  = D_gt.width();
  int32_t height = D_gt.height();

  // init errors
  vector<float> errors;
  for (int32_t i=0; i<2*MAX_ERROR; i++)
    errors.push_back(0);
  int32_t num_pixels = 0;
  int32_t num_pixels_result = 0;

  // for all pixels do
  for (int32_t u=0; u<width; u++) {
    for (int32_t v=0; v<height; v++) {
      if (D_gt.isValid(u,v)) {
        float d_err  = fabs(D_gt.getDisp(u,v)-D_ipol.getDisp(u,v));
        for (int32_t i=0; i<MAX_ERROR; i++)
          if (d_err>(float)(i+1))
            errors[i*2+0]++;
        num_pixels++;
        if (D_orig.isValid(u,v)) {
          for (int32_t i=0; i<MAX_ERROR; i++)
            if (d_err>(float)(i+1))
              errors[i*2+1]++;
          num_pixels_result++;
        }
      }
    }
  }

  // check number of pixels
  if (num_pixels==0) {
    cout << "ERROR: Ground truth defect => Please write me an email!" << endl;
    throw 1;
  }

  // normalize errors
  for (int32_t i=0; i<errors.size(); i+=2)
    errors[i] /= max((float)num_pixels,1.0f);
  if (num_pixels_result>0)
    for (int32_t i=1; i<errors.size(); i+=2)
      errors[i] /= max((float)num_pixels_result,1.0f);

  // push back density
  errors.push_back((float)num_pixels_result/max((float)num_pixels,1.0f));

  // return errors
  return errors;
}

vector<float> disparityErrorsAverage (DisparityImage &D_gt,DisparityImage &D_orig,DisparityImage &D_ipol) {

  // check file size
  if (D_gt.width()!=D_orig.width() || D_gt.height()!=D_orig.height()) {
    cout << "ERROR: Wrong file size!" << endl;
    throw 1;
  }

  // extract width and height
  int32_t width  = D_gt.width();
  int32_t height = D_gt.height();

  // init errors
  vector<float> errors;
  for (int32_t i=0; i<2; i++)
    errors.push_back(0);
  int32_t num_pixels = 0;
  int32_t num_pixels_result = 0;

  // for all pixels do
  for (int32_t u=0; u<width; u++) {
    for (int32_t v=0; v<height; v++) {
      if (D_gt.isValid(u,v)) {
        float d_err = fabs(D_gt.getDisp(u,v)-D_ipol.getDisp(u,v));
        errors[0] += d_err;
        num_pixels++;
        if (D_orig.isValid(u,v)) {
          errors[1] += d_err;
          num_pixels_result++;
        }
      }
    }
  }

  // normalize errors
  errors[0] /= max((float)num_pixels,1.0f);
  errors[1] /= max((float)num_pixels_result,1.0f);

  // return errors
  return errors;
}

bool eval (string prefix, string result_dir, string gt_noc_dir, string gt_occ_dir) {

  // vector for storing the errors
  vector< vector<float> > errors_noc_out;
  vector< vector<float> > errors_occ_out;
  vector< vector<float> > errors_noc_avg;
  vector< vector<float> > errors_occ_avg;

    
  // output
  // mail->msg("Processing: %s.png",prefix.c_str());

  // load ground truth disparity maps
  DisparityImage D_gt_noc(gt_noc_dir + "/" + prefix + ".png");
  DisparityImage D_gt_occ(gt_occ_dir + "/" + prefix + ".png");

  // check submitted result
  string image_file = result_dir + "/data/" + prefix + ".png";
  if (!imageFormat(image_file,png::color_type_gray,16,D_gt_noc.width(),D_gt_noc.height())) {
    // mail->msg("ERROR: Input must be png, 1 channel, 16 bit, %d x %d px",
    //          D_gt_noc.width(),D_gt_noc.height());
  }

  // load submitted result
  DisparityImage D_orig(image_file);

  // interpolate missing values
  DisparityImage D_ipol(D_orig);
  D_ipol.interpolateBackground();

  // add disparity errors
  vector<float> errors_noc_out_curr = disparityErrorsOutlier(D_gt_noc,D_orig,D_ipol);
  vector<float> errors_occ_out_curr = disparityErrorsOutlier(D_gt_occ,D_orig,D_ipol);
  vector<float> errors_noc_avg_curr = disparityErrorsAverage(D_gt_noc,D_orig,D_ipol);
  vector<float> errors_occ_avg_curr = disparityErrorsAverage(D_gt_occ,D_orig,D_ipol);
  errors_noc_out.push_back(errors_noc_out_curr);
  errors_occ_out.push_back(errors_occ_out_curr);
  errors_noc_avg.push_back(errors_noc_avg_curr);
  errors_occ_avg.push_back(errors_occ_avg_curr);


  // save errors of error images to text file
  FILE *errors_noc_out_file = fopen((result_dir + "/errors_noc_out/" + prefix + ".txt").c_str(),"w");
  FILE *errors_occ_out_file = fopen((result_dir + "/errors_occ_out/" + prefix + ".txt").c_str(),"w");
  FILE *errors_noc_avg_file = fopen((result_dir + "/errors_noc_avg/" + prefix + ".txt").c_str(),"w");
  FILE *errors_occ_avg_file = fopen((result_dir + "/errors_occ_avg/" + prefix + ".txt").c_str(),"w");
  if (errors_noc_out_file==NULL || errors_occ_out_file==NULL ||
      errors_noc_avg_file==NULL || errors_occ_avg_file==NULL) {
    // mail->msg("ERROR: Couldn't generate/store output statistics!");
    return false;
  }
  for (int32_t j=0; j<errors_noc_out_curr.size(); j++) {
    fprintf(errors_noc_out_file,"%f ",errors_noc_out_curr[j]);
    fprintf(errors_occ_out_file,"%f ",errors_occ_out_curr[j]);
  }
  for (int32_t j=0; j<errors_noc_avg_curr.size(); j++) {
    fprintf(errors_noc_avg_file,"%f ",errors_noc_avg_curr[j]);
    fprintf(errors_occ_avg_file,"%f ",errors_occ_avg_curr[j]);
  }
  fprintf(errors_noc_out_file,"\n");
  fprintf(errors_occ_out_file,"\n");
  fprintf(errors_noc_avg_file,"\n");
  fprintf(errors_occ_avg_file,"\n");
  fclose(errors_noc_out_file);
  fclose(errors_occ_out_file);
  fclose(errors_noc_avg_file);
  fclose(errors_occ_avg_file);

  // save error image
  png::image<png::rgb_pixel> D_err = D_ipol.errorImage(D_gt_noc,D_gt_occ);
  D_err.write(result_dir + "/errors_img/" + prefix + ".png");

  // compute maximum disparity
  float max_disp = D_gt_occ.maxDisp();

  // save original flow image
  D_orig.writeColor(result_dir + "/disp_orig/" + prefix + ".png",max_disp);

  // save interpolated flow image
  D_ipol.writeColor(result_dir + "/disp_ipol/" + prefix + ".png",max_disp);

}

int32_t evaStereoMain (int32_t argc,char *argv[]) {

  // we need 6 arguments
  if (argc!=5) {
    cout << "Usage: ./eval_flow prefix result_dir gt_noc_dir gt_occ_dir" << endl;
    return 1;
  }

  // read arguments
  string prefix = string(argv[1])+"_10";
  string  result_dir = argv[2];
  string  gt_noc_dir = argv[3];
  string  gt_occ_dir = argv[4];
  
  // run evaluation
  eval(prefix, result_dir, gt_noc_dir, gt_occ_dir);

  return 0;
}

