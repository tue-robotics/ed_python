from test_tools.doctest_unittest import _TestDocTests
import unittest


class DocTestsED(_TestDocTests):
    def __init__(self, method_name="test_doctests"):
        super(DocTestsED, self).__init__(pkg_name="ed_python", module_name="ed", method_name=method_name)


if __name__ == "__main__":
    suite = unittest.TestSuite()
    suite.addTest(DocTestsED())
    unittest.TextTestRunner(verbosity=2).run(suite)
