function helperTMTestCasePostProcessing(logsout,results,sltest_testCase,testCaseID)
% helperTMTestCasePostProcessing performs the post-processing of the test results in
% the Automate Testing of Highway Lane Following example. It updates the
% description of the test result with the birds-eye plot video and
% generates the plots and video results. 

% Copyright 2023 The MathWorks, Inc.


% Generate Plots & Video Results
helperGenerateFilesForLaneFollowingReport(logsout);

% Add Birds-Eye Plot link to description. 
videoFileName = convertCharsToStrings(sltest_testCase.Name) + ".avi";

currentResultset = results(end); 

%Get the test cases object based on the the hierarchy of execution between
%test file, test suite and test case. 
resultsFromTestCase = currentResultset.getTestFileResults.getTestSuiteResults.getTestCaseResults';
[numResultsFromTestFile,~] = size(resultsFromTestCase);

if (numResultsFromTestFile == 0 )
    resultsFromTestCase = currentResultset.getTestSuiteResults.getTestCaseResults';
    [numResultsFromTestSuite,~] = size(resultsFromTestCase);
end

if (numResultsFromTestFile == 0 && numResultsFromTestSuite== 0)
    resultsFromTestCase = currentResultset.getTestCaseResults';
end

%Update the description with hyperlink. 
resultsFromTestCase(testCaseID).Description = "<a href=""matlab:implay('" +  videoFileName + "')"">" + videoFileName + "</a>";

end