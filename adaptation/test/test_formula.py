"""
Unit tests for Formula module using pytest
"""

import pytest
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

# Correct import path
from adaptation.model.formula import Formula, FormulaError


class TestFormula:
    """Test class for Formula functionality"""
    
    def test_simple_construct(self):
        """Test simple formula construction"""
        # Should not raise any exception
        formula = Formula("x+y")
        assert formula is not None
    
    def test_construct_with_terms_and_values(self):
        """Test construction with terms and values"""
        # Should not raise any exception
        formula = Formula("x+y", ["x", "y"], [1, 2])
        assert formula is not None
    
    def test_invalid_formula_construction(self):
        """Test invalid formula construction"""
        with pytest.raises(ValueError):
            Formula("x+y", ["x", "y", "z"], [1, 2])
    
    def test_set_and_get_terms_value_map(self):
        """Test setting and getting term-value map"""
        formula = Formula("x+y")
        terms = ["x", "y"]
        values = [1, 2]
        expected_map = {"x": 1, "y": 2}
        
        formula.set_term_value_map(terms, values)
        returned_map = formula.get_term_value_map()
        
        assert returned_map == expected_map
    
    def test_evaluate_one_term_formula(self):
        """Test evaluation of single term formula"""
        formula = Formula("x+x", ["x"], [2])
        
        answer = formula.evaluate()
        assert answer == 4
    
    def test_evaluate_two_term_formula(self):
        """Test evaluation of two term formula"""
        formula = Formula("x+y", ["x", "y"], [1, 2])
        
        answer = formula.evaluate()
        assert answer == 3
    
    def test_evaluate_nonexistent_term(self):
        """Test evaluation with missing terms"""
        formula = Formula("x+y")
        
        with pytest.raises(FormulaError):
            formula.evaluate()
    
    def test_get_terms(self):
        """Test getting formula terms"""
        formula = Formula("x+y")
        expected_terms = {"x", "y"}
        
        returned_terms = set(formula.get_terms())
        
        assert returned_terms == expected_terms
    
    def test_complex_formula(self):
        """Test complex mathematical formula"""
        # Test BSN-like reliability formula
        formula = Formula("R_G3_T1_1 * R_G3_T1_2 * R_G3_T1_3")
        formula.set_term_value_map(
            ["R_G3_T1_1", "R_G3_T1_2", "R_G3_T1_3"],
            [0.95, 0.92, 0.88]
        )
        
        result = formula.evaluate()
        expected = 0.95 * 0.92 * 0.88
        
        assert abs(result - expected) < 1e-6
    
    def test_cost_formula(self):
        """Test BSN-like cost formula"""
        formula = Formula("W_G3_T1_1 + W_G3_T1_2 + W_G3_T1_3")
        formula.set_term_value_map(
            ["W_G3_T1_1", "W_G3_T1_2", "W_G3_T1_3"],
            [10.5, 15.2, 8.7]
        )
        
        result = formula.evaluate()
        expected = 10.5 + 15.2 + 8.7
        
        assert abs(result - expected) < 1e-6
    
    def test_mathematical_functions(self):
        """Test mathematical functions"""
        formula = Formula("sin(x) + cos(y)")
        formula.set_term_value_map(["x", "y"], [0, 0])
        
        result = formula.evaluate()
        expected = 0 + 1  # sin(0) + cos(0)
        
        assert abs(result - expected) < 1e-6
    
    def test_constants(self):
        """Test mathematical constants"""
        formula = Formula("pi * 2")
        
        result = formula.evaluate()
        expected = 3.14159265359 * 2
        
        assert abs(result - expected) < 1e-6


# Additional pytest-specific tests you can add:

class TestFormulaParametrized:
    """Parametrized tests for Formula functionality"""
    
    @pytest.mark.parametrize("formula_str,terms,values,expected", [
        ("x+y", ["x", "y"], [1, 2], 3),
        ("x*y", ["x", "y"], [3, 4], 12),
        ("x-y", ["x", "y"], [10, 3], 7),
        ("x/y", ["x", "y"], [8, 2], 4),
    ])
    def test_basic_operations(self, formula_str, terms, values, expected):
        """Test basic mathematical operations"""
        formula = Formula(formula_str, terms, values)
        result = formula.evaluate()
        assert result == expected
    
    @pytest.mark.parametrize("invalid_terms,invalid_values", [
        (["x", "y"], [1]),  # Mismatched lengths
        (["x"], [1, 2]),    # Mismatched lengths
        ([], [1]),          # Empty terms with values
    ])
    def test_invalid_term_value_combinations(self, invalid_terms, invalid_values):
        """Test invalid term-value combinations"""
        with pytest.raises(ValueError):
            Formula("x+y", invalid_terms, invalid_values)


@pytest.fixture
def sample_formula():
    """Fixture providing a sample formula for tests"""
    return Formula("x+y+z", ["x", "y", "z"], [1, 2, 3])


class TestFormulaWithFixtures:
    """Tests using pytest fixtures"""
    
    def test_formula_fixture(self, sample_formula):
        """Test using the sample formula fixture"""
        result = sample_formula.evaluate()
        assert result == 6
    
    def test_modify_formula_values(self, sample_formula):
        """Test modifying formula values"""
        sample_formula.set_term_value_map(["x", "y", "z"], [2, 4, 6])
        result = sample_formula.evaluate()
        assert result == 12


# Error handling tests
class TestFormulaErrorHandling:
    """Test error handling in Formula"""
    
    def test_division_by_zero(self):
        """Test division by zero handling"""
        formula = Formula("x/y", ["x", "y"], [1, 0])
        
        with pytest.raises((ZeroDivisionError, FormulaError)):
            formula.evaluate()
    # changed test from x + + y to x + * y
    def test_invalid_syntax(self):
        """Test invalid formula syntax"""
        with pytest.raises((ValueError, FormulaError)):
            Formula("x + * y")
    
    def test_undefined_variable(self):
        """Test undefined variable in formula"""
        formula = Formula("x + undefined_var")
        formula.set_term_value_map(["x"], [5])
        
        with pytest.raises(FormulaError):
            formula.evaluate()


# Performance/edge case tests
class TestFormulaEdgeCases:
    """Test edge cases and performance"""
    
    def test_empty_formula(self):
        """Test empty formula string"""
        with pytest.raises((ValueError, FormulaError)):
            Formula("")
    
    def test_large_numbers(self):
        """Test with large numbers"""
        formula = Formula("x + y", ["x", "y"], [1e10, 2e10])
        result = formula.evaluate()
        assert result == 3e10
    
    def test_very_small_numbers(self):
        """Test with very small numbers"""
        formula = Formula("x + y", ["x", "y"], [1e-10, 2e-10])
        result = formula.evaluate()
        assert abs(result - 3e-10) < 1e-15
    
    def test_many_terms(self):
        """Test formula with many terms"""
        terms = [f"x{i}" for i in range(100)]
        values = [1] * 100
        formula_str = " + ".join(terms)
        
        formula = Formula(formula_str, terms, values)
        result = formula.evaluate()
        assert result == 100