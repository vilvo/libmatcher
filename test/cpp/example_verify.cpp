/*
    This is a sample code to preset how to use Matcher library.
 */

#include <iostream>
#include "../../Matcher.hpp"
#include "../../MatchEntry.hpp"
#include "../../MatcherUtils.hpp"

std::string SCREENSHOT = "../img/homescreen.png";

void printresults(const MatchResult *mresult) {
    std::cout << "Result: " << mresult->result[0] << std::endl;
    std::cout << "result image: " << mresult->resultimage << std::endl;
    std::cout << "message: " << mresult->message << std::endl;
}

int main() {
    // First of all we need to have Matcher configuration
    matcher::MatcherConfig mconfig;
    mconfig.useOCR = false;
    mconfig.useOIR = true;
    mconfig.nfeatures = 1000;
    mconfig.scaleFactor = 2.0f;  // for > 1.4f GPU ORB fails
    mconfig.nlevels = 10;
    mconfig.edgeThreshold = 31;
    mconfig.firstLevel = 0;
    mconfig.WTA_K = 2;
    mconfig.scoreType = 0;
    mconfig.patchSize = 31;
    mconfig.min_dist_thresh = 0;
    mconfig.max_nndr_ratio = 1;
    mconfig.ransac_reprojection_thresh = 10.0f;
    mconfig.min_inliers = 8;
    mconfig.gpu_enabled = false;
    mconfig.scale_invariant = false;
    mconfig.locate = false;

    cv::setBreakOnError(true); // helps with gdb to get bt

    // Now we can create Matcher object
    cv::Ptr<Matcher> matcher = new Matcher(mconfig);

    // Load screenshot and icon image
    cv::Mat image = cv::imread(SCREENSHOT);
    cv::Ptr<MatchEntry> entry = new MatchEntry(cv::imread(SCREENSHOT));

    // Compute keypoints and descriptors for feature matching
    matcher->computeMatchEntry(entry);

    // Create match query
    MatchQuery mquery;
    mquery.screenshot = SCREENSHOT;
    mquery.icon = SCREENSHOT;
    mquery.threshold = 0;

    // Match image to itself with default method
    MatchResult *mresult = new MatchResult();
    cv::Ptr<cv::Mat> resultimg = matcher->match(image, mquery, mresult, entry);

    // Save result image
    matcher->drawMatchDetails(mquery, mresult, resultimg);
    MatcherUtils::saveResultImage(resultimg, mquery, mresult, SCREENSHOT);

    // Print results
    printresults(mresult);
    delete mresult;
    return 0;
}