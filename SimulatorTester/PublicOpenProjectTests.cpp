#include "CppUnitTest.h"
#include "OpenProjectSimulator.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace SimulatorTester
{		
	TEST_CLASS(PublicOpenProjectTests)
	{
	public: 
		void testSceneSetup(OpenProjectSimulator* &ops) {
			if (ops) delete ops;
			ops = new OpenProjectSimulator();
		}

		TEST_METHOD(SampleTest)
		{
			OpenProjectSimulator * ops = NULL;
			testSceneSetup(ops);
			int num = 2;
			Assert::AreEqual(2.0f,(float)num,0.0001f,L"2 is not equal to 2",LINE_INFO());
		}
	};
}