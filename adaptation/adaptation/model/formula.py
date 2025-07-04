import re
import ast
import operator
import math
from typing import Dict, List, Union, Any
from collections import defaultdict

class FormulaError(Exception):
    """Custom exception for Formula-related errors."""
    
    pass


class Formula:
    """
    Python implementation of BSN Formula class.
    
    Provides mathematical expression parsing and evaluation with support
    for variables, constants, and mathematical functions.
    
    Attributes:
        _expression_text: The original mathematical expression string
        _term_value: Dictionary mapping variable names to values
        _parsed_expression: AST representation of the expression
        _variables: Set of variable names found in the expression
    
    Examples:
        Basic usage:
        ```python
        formula = Formula("x + y", ["x", "y"], [1, 2])
        result = formula.evaluate()  # Returns 3
        ```
        
        Using mathematical functions:
        ```python
        formula = Formula("sin(x) + cos(y)")
        formula.set_term_value_map(["x", "y"], [0, 0])
        result = formula.evaluate()  # Returns 1
        ```
    """
    
    # Supported mathematical operations
    _operators = {
        ast.Add: operator.add,
        ast.Sub: operator.sub,
        ast.Mult: operator.mul,
        ast.Div: operator.truediv,
        ast.Pow: operator.pow,
        ast.BitXor: operator.xor,
        ast.USub: operator.neg,
        ast.UAdd: operator.pos,
    }
    
    # Supported mathematical functions
    _functions = {
        'sin': math.sin,
        'cos': math.cos,
        'tan': math.tan,
        'asin': math.asin,
        'acos': math.acos,
        'atan': math.atan,
        'sinh': math.sinh,
        'cosh': math.cosh,
        'tanh': math.tanh,
        'exp': math.exp,
        'log': math.log,
        'log10': math.log10,
        'sqrt': math.sqrt,
        'abs': abs,
        'ceil': math.ceil,
        'floor': math.floor,
        'pow': pow,
        'min': min,
        'max': max,
    }
    
    # Mathematical constants
    _constants = {
        'pi': math.pi,
        'e': math.e,
        'tau': math.tau,
    }

    def __init__(self, text: str = "", terms: List[str] = None, values: List[float] = None):
        """
        Initialize Formula.
        
        Args:
            text: Mathematical expression as string
            terms: List of variable names
            values: List of values corresponding to terms
        """
        self._expression_text = text
        self._term_value = {}
        self._parsed_expression = None
        self._variables = set()
        
        # Check for empty formula and raise ValueError
        if text is not None and text.strip() == "":
            raise ValueError("Empty formula expression")
        
        if text and text.strip():
            self._parse_expression(text)
        
        if terms is not None and values is not None:
            self.set_term_value_map(terms, values)
    
    def __del__(self):
        """Destructor."""
        pass
    
    def __copy__(self):
        """Copy constructor."""
        new_formula = Formula(self._expression_text)
        new_formula._term_value = self._term_value.copy()
        return new_formula
    
    def __deepcopy__(self, memo):
        """Deep copy constructor."""
        return self.__copy__()
    
    def _parse_expression(self, text: str):
        """Parse mathematical expression string."""
        if not text or text.strip() == "":
            raise ValueError("Empty formula expression")
            
        try:
            # Clean the expression
            cleaned_text = self._clean_expression(text)
            
            # Parse using AST
            self._parsed_expression = ast.parse(cleaned_text, mode='eval')
            
            # Extract variables
            self._variables = self._extract_variables(self._parsed_expression)
            
        except SyntaxError as e:
            # Raise ValueError for syntax errors to match test expectations
            raise ValueError(f"Invalid mathematical expression: {text[:20]}...")
    
    def _clean_expression(self, text: str) -> str:
        """Clean and prepare expression for parsing."""
        # Replace ^ with ** for power operations
        text = text.replace('^', '**')
        
        # MUCH more conservative approach - only handle very specific cases
        # Don't touch variable names with underscores or function calls
        
        # Only add multiplication for simple cases like "2x" -> "2*x"
        # But NOT for cases like "sin" or "R_G3_T1_1"
        text = re.sub(r'(\d)\s*([a-zA-Z][a-zA-Z0-9_]*)', r'\1*\2', text)
        
        return text
    
    def _extract_variables(self, node) -> set:
        """Extract variable names from AST."""
        variables = set()
        
        for child in ast.walk(node):
            if isinstance(child, ast.Name) and isinstance(child.ctx, ast.Load):
                var_name = child.id
                # Skip if it's a function or constant
                if var_name not in self._functions and var_name not in self._constants:
                    variables.add(var_name)
        
        return variables
    
    def _evaluate_node(self, node, variables: Dict[str, float]):
        """Evaluate AST node recursively."""
        if isinstance(node, ast.Constant):  # Python 3.8+
            return node.value
        elif isinstance(node, ast.Num):  # Python < 3.8
            return node.n
        elif isinstance(node, ast.Name):
            var_name = node.id
            if var_name in variables:
                return variables[var_name]
            elif var_name in self._constants:
                return self._constants[var_name]
            elif var_name in self._functions:
                return self._functions[var_name]
            else:
                raise FormulaError(f"Unknown variable or function: {var_name}")
        elif isinstance(node, ast.BinOp):
            left = self._evaluate_node(node.left, variables)
            right = self._evaluate_node(node.right, variables)
            return self._operators[type(node.op)](left, right)
        elif isinstance(node, ast.UnaryOp):
            operand = self._evaluate_node(node.operand, variables)
            return self._operators[type(node.op)](operand)
        elif isinstance(node, ast.Call):
            func_name = node.func.id
            if func_name in self._functions:
                args = [self._evaluate_node(arg, variables) for arg in node.args]
                return self._functions[func_name](*args)
            else:
                raise FormulaError(f"Unknown function: {func_name}")
        else:
            raise FormulaError(f"Unsupported operation: {type(node)}")
    
    def get_expression_text(self) -> str:
        """Get the original expression text."""
        return self._expression_text
    
    def set_expression(self, text: str):
        """Set new expression."""
        self._expression_text = text
        if text and text.strip():
            self._parse_expression(text)
        elif text is not None and text.strip() == "":
            raise ValueError("Empty formula expression")
    
    def get_term_value_map(self) -> Dict[str, float]:
        """Get term-value mapping."""
        return self._term_value.copy()
    
    def set_term_value_map(self, terms: List[str], values: List[float]):
        """Set term-value mapping from lists."""
        if len(terms) != len(values):
            raise ValueError("Terms and values size do not correspond to each other.")
        
        self._term_value = {term: value for term, value in zip(terms, values)}
    
    def set_term_value_map_dict(self, term_value: Dict[str, float]):
        """Set term-value mapping from dictionary."""
        self._term_value = term_value.copy()
    
    def evaluate(self) -> float:
        """Evaluate the expression with current term values."""
        if not self._parsed_expression:
            raise FormulaError("No expression to evaluate")
        
        # Combine term values with constants
        all_variables = {**self._constants, **self._term_value}
        
        # Check if all required variables are provided
        missing_vars = self._variables - set(all_variables.keys())
        if missing_vars:
            raise FormulaError(f"Missing values for variables: {missing_vars}")
        
        try:
            return self._evaluate_node(self._parsed_expression.body, all_variables)
        except Exception as e:
            raise FormulaError(f"Error evaluating expression: {str(e)}")
    
    def get_terms(self) -> List[str]:
        """Get all variable terms in the expression."""
        return list(self._variables)
    
    def get_formula(self) -> str:
        """Get formula text (compatibility with original BSN API)."""
        return self._expression_text