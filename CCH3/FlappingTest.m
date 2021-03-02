function tests = FlappingTest
tests = functiontests(localfunctions);
end

function testSign1(testCase)
[beta_0,~,~] = Flapping(0,0,0,deg2rad(10),0,0,10);
verifyGreaterThan(testCase,beta_0,0)
end

function testSign2(testCase)
[~,beta_1c,beta_1s] = Flapping(0,0,0,deg2rad(10),0,0,10);
verifyEqual(testCase,beta_1c,0)
verifyEqual(testCase,beta_1s,0)
end

function testSign3(testCase)
[~,beta_1c,beta_1s] = Flapping(50,0,0,deg2rad(10),0,0,10);
verifyLessThan(testCase,beta_1c,0)
verifyLessThan(testCase,beta_1s,0)
end

