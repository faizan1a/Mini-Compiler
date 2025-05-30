using System;
using System.Collections.Generic;
using System.IO;
using System.Text;
using System.Linq;


namespace MiniCompiler
{
    class Program
    {
        static void Main(string[] args)
        {
            Console.WriteLine("=== Advanced Mini Compiler ===");
            Console.WriteLine("Supports: int, float, bool types with if/else, while, functions");
            Console.WriteLine();

            if (args.Length == 0)
            {
                Console.WriteLine("Interactive mode - Enter your code (type 'EXIT' to quit):");
                Console.WriteLine("Sample: int x = 5; float y = 3.14; if (x > 3) { print x; }");
                Console.WriteLine();

                string input = "";
                string line;
                while ((line = Console.ReadLine()) != "EXIT")
                {
                    input += line + "\n";
                }

                if (!string.IsNullOrEmpty(input))
                {
                    CompileAndRun(input, "Interactive");
                }
            }
            else
            {
                string filename = args[0];
                if (File.Exists(filename))
                {
                    string sourceCode = File.ReadAllText(filename);
                    CompileAndRun(sourceCode, filename);
                }
                else
                {
                    Console.WriteLine($"Error: File '{filename}' not found.");
                }
            }
        }

        // In CompileAndRun, add printing of the actual data for each phase:

        static void CompileAndRun(string sourceCode, string sourceName)
        {
            try
            {
                Console.WriteLine($"\n=== Compiling {sourceName} ===");

                // Phase 1: Lexical Analysis
                Console.WriteLine("Phase 1: Lexical Analysis...");
                var lexer = new Lexer(sourceCode);
                var tokens = lexer.Tokenize();
                Console.WriteLine($"✓ Generated {tokens.Count} tokens");
                foreach (var token in tokens)
                    Console.WriteLine(token);

                // Phase 2: Syntax Analysis (Parsing)
                Console.WriteLine("Phase 2: Syntax Analysis (Parsing)...");
                var parser = new Parser(tokens);
                var ast = parser.Parse();
                Console.WriteLine("✓ Abstract Syntax Tree built successfully");
                PrintAST(ast);

                // Phase 3: Semantic Analysis
                Console.WriteLine("Phase 3: Semantic Analysis...");
                var symbolTable = new SymbolTable();
                var semanticAnalyzer = new SemanticAnalyzer(symbolTable);
                var annotatedAST = semanticAnalyzer.Analyze(ast);
                Console.WriteLine("✓ Semantic analysis completed");
                Console.WriteLine("Symbol Table:");
                foreach (var sym in symbolTable.GetAllSymbols())
                    Console.WriteLine($"  {sym.Name} : {sym.Type} (Scope {sym.Scope}, Addr {sym.Address}, Init {sym.IsInitialized})");

                // Phase 4: Intermediate Code Generation
                Console.WriteLine("Phase 4: Intermediate Code Generation...");
                var irGenerator = new IRGenerator(symbolTable);
                var intermediateCode = irGenerator.Generate(annotatedAST);
                Console.WriteLine($"✓ Generated {intermediateCode.Count} IR instructions");
                foreach (var instr in intermediateCode)
                    Console.WriteLine(instr);

                // Phase 5: Optimization (Basic)
                Console.WriteLine("Phase 5: Code Optimization...");
                var optimizer = new Optimizer();
                var optimizedCode = optimizer.Optimize(intermediateCode);
                Console.WriteLine($"✓ Optimized to {optimizedCode.Count} instructions");
                foreach (var instr in optimizedCode)
                    Console.WriteLine(instr);

                // Phase 6: Target Code Generation (Stack-based VM)
                Console.WriteLine("Phase 6: Target Code Generation...");
                var codeGenerator = new CodeGenerator();
                var targetCode = codeGenerator.Generate(optimizedCode);
                Console.WriteLine($"✓ Generated {targetCode.Count} VM instructions");
                foreach (var instr in targetCode)
                    Console.WriteLine(instr);

                // Phase 7: Execution
                Console.WriteLine("\n=== Execution Output ===");
                var vm = new StackVM();
                vm.Execute(targetCode);

                Console.WriteLine("\n=== Compilation Successful ===");
            }
            catch (CompilerException ex)
            {
                Console.WriteLine($"Compilation Error: {ex.Message}");
                if (ex.Line > 0)
                    Console.WriteLine($"At line {ex.Line}, column {ex.Column}");
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Internal Compiler Error: {ex.Message}");
            }
        }

        // Helper function to print AST (simple indented format)
        static void PrintAST(ASTNode node, int indent = 0)
        {
            string pad = new string(' ', indent * 2);
            switch (node)
            {
                case ProgramNode prog:
                    Console.WriteLine($"{pad}Program");
                    foreach (var stmt in prog.Statements)
                        PrintAST(stmt, indent + 1);
                    break;
                case VarDeclarationNode varDecl:
                    Console.WriteLine($"{pad}VarDecl {varDecl.DeclaredType} {varDecl.Name}");
                    if (varDecl.InitialValue != null)
                        PrintAST(varDecl.InitialValue, indent + 1);
                    break;
                case AssignmentNode assign:
                    Console.WriteLine($"{pad}Assign {assign.Variable}");
                    PrintAST(assign.Value, indent + 1);
                    break;
                case BinaryOpNode bin:
                    Console.WriteLine($"{pad}BinaryOp {bin.Operator}");
                    PrintAST(bin.Left, indent + 1);
                    PrintAST(bin.Right, indent + 1);
                    break;
                case UnaryOpNode unary:
                    Console.WriteLine($"{pad}UnaryOp {unary.Operator}");
                    PrintAST(unary.Operand, indent + 1);
                    break;
                case NumberNode num:
                    Console.WriteLine($"{pad}Number {num.Value}");
                    break;
                case FloatNode fnum:
                    Console.WriteLine($"{pad}Float {fnum.Value}");
                    break;
                case BoolNode b:
                    Console.WriteLine($"{pad}Bool {b.Value}");
                    break;
                case IdentifierNode id:
                    Console.WriteLine($"{pad}Identifier {id.Name}");
                    break;
                case IfNode ifn:
                    Console.WriteLine($"{pad}If");
                    PrintAST(ifn.Condition, indent + 1);
                    Console.WriteLine($"{pad}Then:");
                    PrintAST(ifn.ThenBranch, indent + 2);
                    if (ifn.ElseBranch != null)
                    {
                        Console.WriteLine($"{pad}else:");
                        PrintAST(ifn.ElseBranch, indent + 2);
                    }
                    break;
                case WhileNode wn:
                    Console.WriteLine($"{pad}While");
                    PrintAST(wn.Condition, indent + 1);
                    PrintAST(wn.Body, indent + 1);
                    break;
                case BlockNode block:
                    Console.WriteLine($"{pad}Block");
                    foreach (var stmt in block.Statements)
                        PrintAST(stmt, indent + 1);
                    break;
                case PrintNode pn:
                    Console.WriteLine($"{pad}Print");
                    PrintAST(pn.Expression, indent + 1);
                    break;
                default:
                    Console.WriteLine($"{pad}{node.GetType().Name}");
                    break;
            }
        }


        // Custom Exception for Compiler Errors
        public class CompilerException : Exception
        {
            public int Line { get; }
            public int Column { get; }

            public CompilerException(string message, int line = 0, int column = 0) : base(message)
            {
                Line = line;
                Column = column;
            }
        }

        // Token Types
        public enum TokenType
        {
            // Literals
            NUMBER, FLOAT_NUMBER, IDENTIFIER, STRING, TRUE, FALSE,

            // Data Types
            INT, FLOAT, BOOL, VOID,

            // Keywords
            IF, ELSE, WHILE, FOR, FUNCTION, RETURN, PRINT,

            // Operators
            PLUS, MINUS, MULTIPLY, DIVIDE, MODULO,
            ASSIGN, EQUALS, NOT_EQUALS, LESS_THAN, GREATER_THAN,
            LESS_EQUAL, GREATER_EQUAL, AND, OR, NOT,

            // Delimiters
            SEMICOLON, COMMA, LEFT_PAREN, RIGHT_PAREN,
            LEFT_BRACE, RIGHT_BRACE, DOT,

            // Special
            EOF, NEWLINE
        }

        public class Token
        {
            public TokenType Type { get; set; }
            public string Value { get; set; }
            public int Line { get; set; }
            public int Column { get; set; }

            public Token(TokenType type, string value, int line, int column)
            {
                Type = type;
                Value = value;
                Line = line;
                Column = column;
            }

            public override string ToString()
            {
                return $"{Type}: '{Value}' at ({Line},{Column})";
            }
        }

        // PHASE 1: LEXICAL ANALYSIS
        public class Lexer
        {
            private string _source;
            private int _position;
            private int _line;
            private int _column;
            private Dictionary<string, TokenType> _keywords;

            public Lexer(string source)
            {
                _source = source;
                _position = 0;
                _line = 1;
                _column = 1;

                _keywords = new Dictionary<string, TokenType>
            {
                {"int", TokenType.INT},
                {"float", TokenType.FLOAT},
                {"bool", TokenType.BOOL},
                {"void", TokenType.VOID},
                {"if", TokenType.IF},
                {"else", TokenType.ELSE},
                {"while", TokenType.WHILE},
                {"for", TokenType.FOR},
                {"function", TokenType.FUNCTION},
                {"return", TokenType.RETURN},
                {"print", TokenType.PRINT},
                {"true", TokenType.TRUE},
                {"false", TokenType.FALSE}
            };
            }

            public List<Token> Tokenize()
            {
                var tokens = new List<Token>();

                while (_position < _source.Length)
                {
                    char current = _source[_position];

                    // Skip whitespace
                    if (char.IsWhiteSpace(current))
                    {
                        HandleWhitespace(current);
                        continue;
                    }

                    // Skip comments
                    if (current == '/' && Peek() == '/')
                    {
                        SkipLineComment();
                        continue;
                    }

                    // Numbers (integer and float)
                    if (char.IsDigit(current))
                    {
                        tokens.Add(ReadNumber());
                        continue;
                    }

                    // Identifiers and keywords
                    if (char.IsLetter(current) || current == '_')
                    {
                        tokens.Add(ReadIdentifier());
                        continue;
                    }

                    // String literals
                    if (current == '"')
                    {
                        tokens.Add(ReadString());
                        continue;
                    }

                    // Multi-character operators
                    if (current == '=' && Peek() == '=')
                    {
                        tokens.Add(new Token(TokenType.EQUALS, "==", _line, _column));
                        Advance(2);
                        continue;
                    }

                    if (current == '!' && Peek() == '=')
                    {
                        tokens.Add(new Token(TokenType.NOT_EQUALS, "!=", _line, _column));
                        Advance(2);
                        continue;
                    }

                    if (current == '<' && Peek() == '=')
                    {
                        tokens.Add(new Token(TokenType.LESS_EQUAL, "<=", _line, _column));
                        Advance(2);
                        continue;
                    }

                    if (current == '>' && Peek() == '=')
                    {
                        tokens.Add(new Token(TokenType.GREATER_EQUAL, ">=", _line, _column));
                        Advance(2);
                        continue;
                    }

                    if (current == '&' && Peek() == '&')
                    {
                        tokens.Add(new Token(TokenType.AND, "&&", _line, _column));
                        Advance(2);
                        continue;
                    }

                    if (current == '|' && Peek() == '|')
                    {
                        tokens.Add(new Token(TokenType.OR, "||", _line, _column));
                        Advance(2);
                        continue;
                    }

                    // Single character tokens
                    TokenType? singleCharType = GetSingleCharToken(current);
                    if (singleCharType.HasValue)
                    {
                        tokens.Add(new Token(singleCharType.Value, current.ToString(), _line, _column));
                        Advance();
                        continue;
                    }

                    throw new CompilerException($"Unexpected character '{current}'", _line, _column);
                }

                tokens.Add(new Token(TokenType.EOF, "", _line, _column));
                return tokens;
            }

            private void HandleWhitespace(char c)
            {
                if (c == '\n')
                {
                    _line++;
                    _column = 1;
                }
                else
                {
                    _column++;
                }
                _position++;
            }

            private void SkipLineComment()
            {
                while (_position < _source.Length && _source[_position] != '\n')
                {
                    _position++;
                    _column++;
                }
            }

            private Token ReadNumber()
            {
                var startColumn = _column;
                var value = new StringBuilder();
                bool isFloat = false;

                while (_position < _source.Length && (char.IsDigit(_source[_position]) || _source[_position] == '.'))
                {
                    if (_source[_position] == '.')
                    {
                        if (isFloat)
                            throw new CompilerException("Invalid number format", _line, _column);
                        isFloat = true;
                    }
                    value.Append(_source[_position]);
                    Advance();
                }

                return new Token(isFloat ? TokenType.FLOAT_NUMBER : TokenType.NUMBER,
                               value.ToString(), _line, startColumn);
            }

            private Token ReadIdentifier()
            {
                var startColumn = _column;
                var value = new StringBuilder();

                while (_position < _source.Length &&
                       (char.IsLetterOrDigit(_source[_position]) || _source[_position] == '_'))
                {
                    value.Append(_source[_position]);
                    Advance();
                }

                string identifier = value.ToString();
                TokenType type = _keywords.ContainsKey(identifier) ? _keywords[identifier] : TokenType.IDENTIFIER;

                return new Token(type, identifier, _line, startColumn);
            }

            private Token ReadString()
            {
                var startColumn = _column;
                var value = new StringBuilder();
                Advance(); // Skip opening quote

                while (_position < _source.Length && _source[_position] != '"')
                {
                    if (_source[_position] == '\\')
                    {
                        Advance();
                        if (_position >= _source.Length)
                            throw new CompilerException("Unterminated string literal", _line, startColumn);

                        switch (_source[_position])
                        {
                            case 'n': value.Append('\n'); break;
                            case 't': value.Append('\t'); break;
                            case '\\': value.Append('\\'); break;
                            case '"': value.Append('"'); break;
                            default:
                                throw new CompilerException($"Invalid escape sequence \\{_source[_position]}", _line, _column);
                        }
                    }
                    else
                    {
                        value.Append(_source[_position]);
                    }
                    Advance();
                }

                if (_position >= _source.Length)
                    throw new CompilerException("Unterminated string literal", _line, startColumn);

                Advance(); // Skip closing quote
                return new Token(TokenType.STRING, value.ToString(), _line, startColumn);
            }

            private TokenType? GetSingleCharToken(char c)
            {
                return c switch
                {
                    '+' => TokenType.PLUS,
                    '-' => TokenType.MINUS,
                    '*' => TokenType.MULTIPLY,
                    '/' => TokenType.DIVIDE,
                    '%' => TokenType.MODULO,
                    '=' => TokenType.ASSIGN,
                    '<' => TokenType.LESS_THAN,
                    '>' => TokenType.GREATER_THAN,
                    '!' => TokenType.NOT,
                    ';' => TokenType.SEMICOLON,
                    ',' => TokenType.COMMA,
                    '(' => TokenType.LEFT_PAREN,
                    ')' => TokenType.RIGHT_PAREN,
                    '{' => TokenType.LEFT_BRACE,
                    '}' => TokenType.RIGHT_BRACE,
                    '.' => TokenType.DOT,
                    _ => null
                };
            }

            private char Peek(int offset = 1)
            {
                int pos = _position + offset;
                if (pos >= _source.Length)
                    return '\0';
                return _source[pos];
            }

            private void Advance(int count = 1)
            {
                for (int i = 0; i < count && _position < _source.Length; i++)
                {
                    _position++;
                    _column++;
                }
            }
        }

        // PHASE 7: SYMBOL TABLE MANAGEMENT
        public enum DataType
        {
            INT, FLOAT, BOOL, VOID, UNKNOWN
        }

        public class Symbol
        {
            public string Name { get; set; }
            public DataType Type { get; set; }
            public int Scope { get; set; }
            public int Address { get; set; }
            public bool IsInitialized { get; set; }

            public Symbol(string name, DataType type, int scope, int address)
            {
                Name = name;
                Type = type;
                Scope = scope;
                Address = address;
                IsInitialized = false;
            }
        }

        public class SymbolTable
        {
            private List<Dictionary<string, Symbol>> _scopes;
            private int _currentScope;
            private int _nextAddress;

            public SymbolTable()
            {
                _scopes = new List<Dictionary<string, Symbol>>();
                _currentScope = 0;
                _nextAddress = 0;
                EnterScope(); // Global scope
            }

            public void EnterScope()
            {
                _scopes.Add(new Dictionary<string, Symbol>());
                _currentScope = _scopes.Count - 1;
            }

            public void ExitScope()
            {
                if (_currentScope > 0)
                {
                    _scopes.RemoveAt(_currentScope);
                    _currentScope = _scopes.Count - 1;
                }
            }

            public Symbol DeclareVariable(string name, DataType type)
            {
                if (_scopes[_currentScope].ContainsKey(name))
                    throw new CompilerException($"Variable '{name}' already declared in current scope");

                var symbol = new Symbol(name, type, _currentScope, _nextAddress++);
                _scopes[_currentScope][name] = symbol;
                return symbol;
            }

            public Symbol LookupVariable(string name)
            {
                for (int i = _currentScope; i >= 0; i--)
                {
                    if (_scopes[i].ContainsKey(name))
                        return _scopes[i][name];
                }
                return null;
            }

            public void MarkInitialized(string name)
            {
                var symbol = LookupVariable(name);
                if (symbol != null)
                    symbol.IsInitialized = true;
            }

            public List<Symbol> GetAllSymbols()
            {
                var allSymbols = new List<Symbol>();
                foreach (var scope in _scopes)
                {
                    allSymbols.AddRange(scope.Values);
                }
                return allSymbols;
            }
        }

        // AST Node Definitions
        public abstract class ASTNode
        {
            public DataType Type { get; set; } = DataType.UNKNOWN;
            public int Line { get; set; }
            public int Column { get; set; }
        }

        public class ProgramNode : ASTNode
        {
            public List<ASTNode> Statements { get; set; } = new List<ASTNode>();
        }

        public class VarDeclarationNode : ASTNode
        {
            public string Name { get; set; }
            public DataType DeclaredType { get; set; }
            public ASTNode InitialValue { get; set; }
        }

        public class AssignmentNode : ASTNode
        {
            public string Variable { get; set; }
            public ASTNode Value { get; set; }
        }

        public class BinaryOpNode : ASTNode
        {
            public ASTNode Left { get; set; }
            public TokenType Operator { get; set; }
            public ASTNode Right { get; set; }
        }

        public class UnaryOpNode : ASTNode
        {
            public TokenType Operator { get; set; }
            public ASTNode Operand { get; set; }
        }

        public class NumberNode : ASTNode
        {
            public int Value { get; set; }
        }

        public class FloatNode : ASTNode
        {
            public float Value { get; set; }
        }

        public class BoolNode : ASTNode
        {
            public bool Value { get; set; }
        }

        public class IdentifierNode : ASTNode
        {
            public string Name { get; set; }
        }

        public class IfNode : ASTNode
        {
            public ASTNode Condition { get; set; }
            public ASTNode ThenBranch { get; set; }
            public ASTNode ElseBranch { get; set; }
        }

        public class WhileNode : ASTNode
        {
            public ASTNode Condition { get; set; }
            public ASTNode Body { get; set; }
        }

        public class BlockNode : ASTNode
        {
            public List<ASTNode> Statements { get; set; } = new List<ASTNode>();
        }

        public class PrintNode : ASTNode
        {
            public ASTNode Expression { get; set; }
        }

        // PHASE 2: SYNTAX ANALYSIS (PARSING)
        public class Parser
        {
            private List<Token> _tokens;
            private int _current;

            public Parser(List<Token> tokens)
            {
                _tokens = tokens;
                _current = 0;
            }

            public ProgramNode Parse()
            {
                var program = new ProgramNode();

                while (!IsAtEnd())
                {
                    var stmt = ParseStatement();
                    if (stmt != null)
                        program.Statements.Add(stmt);
                }

                return program;
            }

            private ASTNode ParseStatement()
            {
                try
                {
                    if (Match(TokenType.INT, TokenType.FLOAT, TokenType.BOOL))
                        return ParseVarDeclaration();

                    if (Match(TokenType.IF))
                        return ParseIfStatement();

                    if (Match(TokenType.WHILE))
                        return ParseWhileStatement();

                    if (Match(TokenType.PRINT))
                        return ParsePrintStatement();

                    if (Match(TokenType.LEFT_BRACE))
                        return ParseBlock();

                    if (Check(TokenType.IDENTIFIER))
                        return ParseAssignment();

                    if (Match(TokenType.SEMICOLON))
                        return null; // Empty statement

                    throw new CompilerException($"Unexpected token: {Peek().Type}", Peek().Line, Peek().Column);
                }
                catch (CompilerException)
                {
                    throw;
                }
                catch (Exception ex)
                {
                    throw new CompilerException($"Parse error: {ex.Message}", Peek().Line, Peek().Column);
                }
            }

            private VarDeclarationNode ParseVarDeclaration()
            {
                var typeToken = Previous();
                var name = Consume(TokenType.IDENTIFIER, "Expected variable name").Value;

                DataType declaredType = typeToken.Type switch
                {
                    TokenType.INT => DataType.INT,
                    TokenType.FLOAT => DataType.FLOAT,
                    TokenType.BOOL => DataType.BOOL,
                    _ => DataType.UNKNOWN
                };

                ASTNode initialValue = null;
                if (Match(TokenType.ASSIGN))
                {
                    initialValue = ParseExpression();
                }

                Consume(TokenType.SEMICOLON, "Expected ';' after variable declaration");

                return new VarDeclarationNode
                {
                    Name = name,
                    DeclaredType = declaredType,
                    InitialValue = initialValue,
                    Line = typeToken.Line,
                    Column = typeToken.Column
                };
            }

            private AssignmentNode ParseAssignment()
            {
                var nameToken = Advance();
                Consume(TokenType.ASSIGN, "Expected '=' in assignment");
                var value = ParseExpression();
                Consume(TokenType.SEMICOLON, "Expected ';' after assignment");

                return new AssignmentNode
                {
                    Variable = nameToken.Value,
                    Value = value,
                    Line = nameToken.Line,
                    Column = nameToken.Column
                };
            }

            private IfNode ParseIfStatement()
            {
                var ifToken = Previous();
                Consume(TokenType.LEFT_PAREN, "Expected '(' after 'if'");
                var condition = ParseExpression();
                Consume(TokenType.RIGHT_PAREN, "Expected ')' after if condition");

                var thenBranch = ParseStatement();
                ASTNode elseBranch = null;

                if (Match(TokenType.ELSE))
                {
                    elseBranch = ParseStatement();
                }

                return new IfNode
                {
                    Condition = condition,
                    ThenBranch = thenBranch,
                    ElseBranch = elseBranch,
                    Line = ifToken.Line,
                    Column = ifToken.Column
                };
            }

            private WhileNode ParseWhileStatement()
            {
                var whileToken = Previous();
                Consume(TokenType.LEFT_PAREN, "Expected '(' after 'while'");
                var condition = ParseExpression();
                Consume(TokenType.RIGHT_PAREN, "Expected ')' after while condition");
                var body = ParseStatement();

                return new WhileNode
                {
                    Condition = condition,
                    Body = body,
                    Line = whileToken.Line,
                    Column = whileToken.Column
                };
            }

            private PrintNode ParsePrintStatement()
            {
                var printToken = Previous();
                var expression = ParseExpression();
                Consume(TokenType.SEMICOLON, "Expected ';' after print statement");

                return new PrintNode
                {
                    Expression = expression,
                    Line = printToken.Line,
                    Column = printToken.Column
                };
            }

            private BlockNode ParseBlock()
            {
                var blockToken = Previous();
                var statements = new List<ASTNode>();

                while (!Check(TokenType.RIGHT_BRACE) && !IsAtEnd())
                {
                    var stmt = ParseStatement();
                    if (stmt != null)
                        statements.Add(stmt);
                }

                Consume(TokenType.RIGHT_BRACE, "Expected '}' after block");

                return new BlockNode
                {
                    Statements = statements,
                    Line = blockToken.Line,
                    Column = blockToken.Column
                };
            }

            private ASTNode ParseExpression()
            {
                return ParseLogicalOr();
            }

            private ASTNode ParseLogicalOr()
            {
                var expr = ParseLogicalAnd();

                while (Match(TokenType.OR))
                {
                    var op = Previous().Type;
                    var right = ParseLogicalAnd();
                    expr = new BinaryOpNode { Left = expr, Operator = op, Right = right };
                }

                return expr;
            }

            private ASTNode ParseLogicalAnd()
            {
                var expr = ParseEquality();

                while (Match(TokenType.AND))
                {
                    var op = Previous().Type;
                    var right = ParseEquality();
                    expr = new BinaryOpNode { Left = expr, Operator = op, Right = right };
                }

                return expr;
            }

            private ASTNode ParseEquality()
            {
                var expr = ParseComparison();

                while (Match(TokenType.EQUALS, TokenType.NOT_EQUALS))
                {
                    var op = Previous().Type;
                    var right = ParseComparison();
                    expr = new BinaryOpNode { Left = expr, Operator = op, Right = right };
                }

                return expr;
            }

            private ASTNode ParseComparison()
            {
                var expr = ParseTerm();

                while (Match(TokenType.GREATER_THAN, TokenType.GREATER_EQUAL, TokenType.LESS_THAN, TokenType.LESS_EQUAL))
                {
                    var op = Previous().Type;
                    var right = ParseTerm();
                    expr = new BinaryOpNode { Left = expr, Operator = op, Right = right };
                }

                return expr;
            }

            private ASTNode ParseTerm()
            {
                var expr = ParseFactor();

                while (Match(TokenType.MINUS, TokenType.PLUS))
                {
                    var op = Previous().Type;
                    var right = ParseFactor();
                    expr = new BinaryOpNode { Left = expr, Operator = op, Right = right };
                }

                return expr;
            }

            private ASTNode ParseFactor()
            {
                var expr = ParseUnary();

                while (Match(TokenType.DIVIDE, TokenType.MULTIPLY, TokenType.MODULO))
                {
                    var op = Previous().Type;
                    var right = ParseUnary();
                    expr = new BinaryOpNode { Left = expr, Operator = op, Right = right };
                }

                return expr;
            }

            private ASTNode ParseUnary()
            {
                if (Match(TokenType.NOT, TokenType.MINUS))
                {
                    var op = Previous().Type;
                    var expr = ParseUnary();
                    return new UnaryOpNode { Operator = op, Operand = expr };
                }

                return ParsePrimary();
            }

            private ASTNode ParsePrimary()
            {
                if (Match(TokenType.TRUE))
                    return new BoolNode { Value = true };

                if (Match(TokenType.FALSE))
                    return new BoolNode { Value = false };

                if (Match(TokenType.NUMBER))
                {
                    return new NumberNode { Value = int.Parse(Previous().Value) };
                }

                if (Match(TokenType.FLOAT_NUMBER))
                {
                    return new FloatNode { Value = float.Parse(Previous().Value) };
                }

                if (Match(TokenType.IDENTIFIER))
                {
                    return new IdentifierNode { Name = Previous().Value };
                }

                if (Match(TokenType.LEFT_PAREN))
                {
                    var expr = ParseExpression();
                    Consume(TokenType.RIGHT_PAREN, "Expected ')' after expression");
                    return expr;
                }

                throw new CompilerException($"Unexpected token: {Peek().Type}", Peek().Line, Peek().Column);
            }

            private bool Match(params TokenType[] types)
            {
                foreach (var type in types)
                {
                    if (Check(type))
                    {
                        Advance();
                        return true;
                    }
                }
                return false;
            }

            private bool Check(TokenType type)
            {
                if (IsAtEnd()) return false;
                return Peek().Type == type;
            }

            private Token Advance()
            {
                if (!IsAtEnd()) _current++;
                return Previous();
            }

            private bool IsAtEnd()
            {
                return Peek().Type == TokenType.EOF;
            }

            private Token Peek()
            {
                return _tokens[_current];
            }

            private Token Previous()
            {
                return _tokens[_current - 1];
            }

            private Token Consume(TokenType type, string message)
            {
                if (Check(type)) return Advance();
                throw new CompilerException($"{message}. Got {Peek().Type} instead.", Peek().Line, Peek().Column);
            }
        }

        // PHASE 3: SEMANTIC ANALYSIS
        public class SemanticAnalyzer
        {
            private SymbolTable _symbolTable;

            public SemanticAnalyzer(SymbolTable symbolTable)
            {
                _symbolTable = symbolTable;
            }

            public ASTNode Analyze(ProgramNode program)
            {
                foreach (var statement in program.Statements)
                {
                    AnalyzeNode(statement);
                }
                return program;
            }

            private void AnalyzeNode(ASTNode node)
            {
                switch (node)
                {
                    case VarDeclarationNode varDecl:
                        AnalyzeVarDeclaration(varDecl);
                        break;
                    case AssignmentNode assignment:
                        AnalyzeAssignment(assignment);
                        break;
                    case BinaryOpNode binaryOp:
                        AnalyzeBinaryOp(binaryOp);
                        break;
                    case UnaryOpNode unaryOp:
                        AnalyzeUnaryOp(unaryOp);
                        break;
                    case IdentifierNode identifier:
                        AnalyzeIdentifier(identifier);
                        break;
                    case IfNode ifNode:
                        AnalyzeIf(ifNode);
                        break;
                    case WhileNode whileNode:
                        AnalyzeWhile(whileNode);
                        break;
                    case BlockNode block:
                        AnalyzeBlock(block);
                        break;
                    case PrintNode print:
                        AnalyzePrint(print);
                        break;
                    case NumberNode number:
                        number.Type = DataType.INT;
                        break;
                    case FloatNode floatNode:
                        floatNode.Type = DataType.FLOAT;
                        break;
                    case BoolNode boolNode:
                        boolNode.Type = DataType.BOOL;
                        break;
                }
            }

            private void AnalyzeVarDeclaration(VarDeclarationNode node)
            {
                var symbol = _symbolTable.DeclareVariable(node.Name, node.DeclaredType);

                if (node.InitialValue != null)
                {
                    AnalyzeNode(node.InitialValue);
                    if (node.InitialValue.Type != node.DeclaredType)
                    {
                        throw new CompilerException(
                            $"Type mismatch in initialization of variable '{node.Name}'. Expected {node.DeclaredType}, got {node.InitialValue.Type}.",
                            node.Line, node.Column);
                    }
                    symbol.IsInitialized = true;
                }
                node.Type = node.DeclaredType;
            }

            private void AnalyzeAssignment(AssignmentNode node)
            {
                var symbol = _symbolTable.LookupVariable(node.Variable);
                if (symbol == null)
                    throw new CompilerException($"Undeclared variable '{node.Variable}'", node.Line, node.Column);

                AnalyzeNode(node.Value);
                if (symbol.Type != node.Value.Type)
                    throw new CompilerException(
                        $"Type mismatch in assignment to '{node.Variable}'. Expected {symbol.Type}, got {node.Value.Type}.",
                        node.Line, node.Column);

                symbol.IsInitialized = true;
                node.Type = symbol.Type;
            }

            private void AnalyzeBinaryOp(BinaryOpNode node)
            {
                AnalyzeNode(node.Left);
                AnalyzeNode(node.Right);

                switch (node.Operator)
                {
                    case TokenType.PLUS:
                    case TokenType.MINUS:
                    case TokenType.MULTIPLY:
                    case TokenType.DIVIDE:
                    case TokenType.MODULO:
                        if ((node.Left.Type == DataType.INT || node.Left.Type == DataType.FLOAT) &&
                            (node.Right.Type == DataType.INT || node.Right.Type == DataType.FLOAT))
                        {
                            node.Type = (node.Left.Type == DataType.FLOAT || node.Right.Type == DataType.FLOAT)
                                ? DataType.FLOAT
                                : DataType.INT;
                        }
                        else
                        {
                            throw new CompilerException("Arithmetic operations require numeric types", node.Line, node.Column);
                        }
                        break;
                    case TokenType.EQUALS:
                    case TokenType.NOT_EQUALS:
                    case TokenType.LESS_THAN:
                    case TokenType.LESS_EQUAL:
                    case TokenType.GREATER_THAN:
                    case TokenType.GREATER_EQUAL:
                        if ((node.Left.Type == DataType.INT || node.Left.Type == DataType.FLOAT) &&
                            (node.Right.Type == DataType.INT || node.Right.Type == DataType.FLOAT))
                        {
                            node.Type = DataType.BOOL;
                        }
                        else
                        {
                            throw new CompilerException("Comparison operations require numeric types", node.Line, node.Column);
                        }
                        break;
                    case TokenType.AND:
                    case TokenType.OR:
                        if (node.Left.Type == DataType.BOOL && node.Right.Type == DataType.BOOL)
                        {
                            node.Type = DataType.BOOL;
                        }
                        else
                        {
                            throw new CompilerException("Logical operations require boolean types", node.Line, node.Column);
                        }
                        break;
                    default:
                        throw new CompilerException($"Unknown binary operator '{node.Operator}'", node.Line, node.Column);
                }
            }

            private void AnalyzeUnaryOp(UnaryOpNode node)
            {
                AnalyzeNode(node.Operand);
                switch (node.Operator)
                {
                    case TokenType.NOT:
                        if (node.Operand.Type != DataType.BOOL)
                            throw new CompilerException("Logical NOT requires boolean operand", node.Line, node.Column);
                        node.Type = DataType.BOOL;
                        break;
                    case TokenType.MINUS:
                        if (node.Operand.Type != DataType.INT && node.Operand.Type != DataType.FLOAT)
                            throw new CompilerException("Unary minus requires numeric operand", node.Line, node.Column);
                        node.Type = node.Operand.Type;
                        break;
                    default:
                        throw new CompilerException($"Unknown unary operator '{node.Operator}'", node.Line, node.Column);
                }
            }

            private void AnalyzeIdentifier(IdentifierNode node)
            {
                var symbol = _symbolTable.LookupVariable(node.Name);
                if (symbol == null)
                    throw new CompilerException($"Undeclared variable '{node.Name}'", node.Line, node.Column);
                if (!symbol.IsInitialized)
                    throw new CompilerException($"Variable '{node.Name}' might not have been initialized", node.Line, node.Column);
                node.Type = symbol.Type;
            }

            private void AnalyzeIf(IfNode node)
            {
                AnalyzeNode(node.Condition);
                if (node.Condition.Type != DataType.BOOL)
                    throw new CompilerException("If condition must be boolean", node.Line, node.Column);

                _symbolTable.EnterScope();
                AnalyzeNode(node.ThenBranch);
                _symbolTable.ExitScope();

                if (node.ElseBranch != null)
                {
                    _symbolTable.EnterScope();
                    AnalyzeNode(node.ElseBranch);
                    _symbolTable.ExitScope();
                }
            }

            private void AnalyzeWhile(WhileNode node)
            {
                AnalyzeNode(node.Condition);
                if (node.Condition.Type != DataType.BOOL)
                    throw new CompilerException("While condition must be boolean", node.Line, node.Column);

                _symbolTable.EnterScope();
                AnalyzeNode(node.Body);
                _symbolTable.ExitScope();
            }

            private void AnalyzeBlock(BlockNode node)
            {
                _symbolTable.EnterScope();
                foreach (var stmt in node.Statements)
                {
                    AnalyzeNode(stmt);
                }
                _symbolTable.ExitScope();
            }

            private void AnalyzePrint(PrintNode node)
            {
                AnalyzeNode(node.Expression);
                // Any type can be printed
            }
        }

        // PHASE 4: INTERMEDIATE CODE GENERATION (IR)
        public enum IROp
        {
            LOAD_CONST, LOAD_VAR, STORE_VAR,
            ADD, SUB, MUL, DIV, MOD,
            CMP_EQ, CMP_NEQ, CMP_LT, CMP_GT, CMP_LE, CMP_GE,
            AND, OR, NOT,
            JUMP, JUMP_IF_FALSE, LABEL,
            PRINT
        }

        public class IRInstruction
        {
            public IROp Op { get; set; }
            public object Arg1 { get; set; }
            public object Arg2 { get; set; }
            public object Arg3 { get; set; }

            public IRInstruction(IROp op, object arg1 = null, object arg2 = null, object arg3 = null)
            {
                Op = op;
                Arg1 = arg1;
                Arg2 = arg2;
                Arg3 = arg3;
            }

            public override string ToString()
            {
                return $"{Op} {Arg1} {Arg2} {Arg3}".Trim();
            }
        }

        public class IRGenerator
        {
            private SymbolTable _symbolTable;
            private List<IRInstruction> _instructions;
            private int _labelCounter = 0;

            public IRGenerator(SymbolTable symbolTable)
            {
                _symbolTable = symbolTable;
            }

            public List<IRInstruction> Generate(ASTNode node)
            {
                _instructions = new List<IRInstruction>();
                GenerateNode(node);
                return _instructions;
            }

            private void GenerateNode(ASTNode node)
            {
                switch (node)
                {
                    case ProgramNode program:
                        foreach (var stmt in program.Statements)
                            GenerateNode(stmt);
                        break;
                    case VarDeclarationNode varDecl:
                        if (varDecl.InitialValue != null)
                        {
                            GenerateNode(varDecl.InitialValue);
                            var symbol = _symbolTable.LookupVariable(varDecl.Name);
                            _instructions.Add(new IRInstruction(IROp.STORE_VAR, symbol.Address));
                        }
                        break;
                    case AssignmentNode assign:
                        GenerateNode(assign.Value);
                        var symbolA = _symbolTable.LookupVariable(assign.Variable);
                        _instructions.Add(new IRInstruction(IROp.STORE_VAR, symbolA.Address));
                        break;
                    case BinaryOpNode binOp:
                        GenerateNode(binOp.Left);
                        GenerateNode(binOp.Right);
                        _instructions.Add(new IRInstruction(MapBinaryOp(binOp.Operator)));
                        break;
                    case UnaryOpNode unaryOp:
                        GenerateNode(unaryOp.Operand);
                        _instructions.Add(new IRInstruction(MapUnaryOp(unaryOp.Operator)));
                        break;
                    case NumberNode num:
                        _instructions.Add(new IRInstruction(IROp.LOAD_CONST, num.Value));
                        break;
                    case FloatNode fnum:
                        _instructions.Add(new IRInstruction(IROp.LOAD_CONST, fnum.Value));
                        break;
                    case BoolNode bnode:
                        _instructions.Add(new IRInstruction(IROp.LOAD_CONST, bnode.Value));
                        break;
                    case IdentifierNode id:
                        var symbolId = _symbolTable.LookupVariable(id.Name);
                        _instructions.Add(new IRInstruction(IROp.LOAD_VAR, symbolId.Address));
                        break;
                    case IfNode ifNode:
                        {
                            GenerateNode(ifNode.Condition);
                            string elseLabel = NewLabel("else");
                            string endLabel = NewLabel("endif");
                            _instructions.Add(new IRInstruction(IROp.JUMP_IF_FALSE, elseLabel));
                            GenerateNode(ifNode.ThenBranch);
                            _instructions.Add(new IRInstruction(IROp.JUMP, endLabel));
                            _instructions.Add(new IRInstruction(IROp.LABEL, elseLabel));
                            if (ifNode.ElseBranch != null)
                                GenerateNode(ifNode.ElseBranch);
                            _instructions.Add(new IRInstruction(IROp.LABEL, endLabel));
                        }
                        break;
                    case WhileNode whileNode:
                        {
                            string startLabel = NewLabel("while_start");
                            string endLabel = NewLabel("while_end");
                            _instructions.Add(new IRInstruction(IROp.LABEL, startLabel));
                            GenerateNode(whileNode.Condition);
                            _instructions.Add(new IRInstruction(IROp.JUMP_IF_FALSE, endLabel));
                            GenerateNode(whileNode.Body);
                            _instructions.Add(new IRInstruction(IROp.JUMP, startLabel));
                            _instructions.Add(new IRInstruction(IROp.LABEL, endLabel));
                        }
                        break;
                    case BlockNode block:
                        foreach (var stmt in block.Statements)
                            GenerateNode(stmt);
                        break;
                    case PrintNode print:
                        GenerateNode(print.Expression);
                        _instructions.Add(new IRInstruction(IROp.PRINT));
                        break;
                }
            }

            private IROp MapBinaryOp(TokenType op)
            {
                return op switch
                {
                    TokenType.PLUS => IROp.ADD,
                    TokenType.MINUS => IROp.SUB,
                    TokenType.MULTIPLY => IROp.MUL,
                    TokenType.DIVIDE => IROp.DIV,
                    TokenType.MODULO => IROp.MOD,
                    TokenType.EQUALS => IROp.CMP_EQ,
                    TokenType.NOT_EQUALS => IROp.CMP_NEQ,
                    TokenType.LESS_THAN => IROp.CMP_LT,
                    TokenType.GREATER_THAN => IROp.CMP_GT,
                    TokenType.LESS_EQUAL => IROp.CMP_LE,
                    TokenType.GREATER_EQUAL => IROp.CMP_GE,
                    TokenType.AND => IROp.AND,
                    TokenType.OR => IROp.OR,
                    _ => throw new CompilerException($"Unknown binary operator '{op}'")
                };
            }

            private IROp MapUnaryOp(TokenType op)
            {
                return op switch
                {
                    TokenType.NOT => IROp.NOT,
                    TokenType.MINUS => IROp.SUB, // For unary minus, will need to handle as 0 - x
                    _ => throw new CompilerException($"Unknown unary operator '{op}'")
                };
            }

            private string NewLabel(string prefix)
            {
                return $"{prefix}_{_labelCounter++}";
            }
        }

        // PHASE 5: OPTIMIZATION (BASIC)
        public class Optimizer
        {
            public List<IRInstruction> Optimize(List<IRInstruction> code)
            {
                // Simple optimization: remove redundant LOAD/STORE pairs, constant folding, etc.
                // For now, just return the code as-is.
                return code;
            }
        }

        // PHASE 6: TARGET CODE GENERATION (STACK-BASED VM)
        public class CodeGenerator
        {
            public List<IRInstruction> Generate(List<IRInstruction> ir)
            {
                // For this simple VM, IR is already close to target code.
                // In a real compiler, this would translate IR to VM instructions.
                return ir;
            }
        }

        // PHASE 7: EXECUTION (STACK-BASED VM)
        public class StackVM
        {
            private object[] _memory = new object[1024];
            private Stack<object> _stack = new Stack<object>();
            private Dictionary<string, int> _labels = new Dictionary<string, int>();

            public void Execute(List<IRInstruction> code)
            {
                // First pass: resolve labels
                for (int i = 0; i < code.Count; i++)
                {
                    if (code[i].Op == IROp.LABEL && code[i].Arg1 is string label)
                    {
                        _labels[label] = i;
                    }
                }

                for (int ip = 0; ip < code.Count; ip++)
                {
                    var instr = code[ip];
                    switch (instr.Op)
                    {
                        case IROp.LOAD_CONST:
                            _stack.Push(instr.Arg1);
                            break;
                        case IROp.LOAD_VAR:
                            _stack.Push(_memory[Convert.ToInt32(instr.Arg1)]);
                            break;
                        case IROp.STORE_VAR:
                            _memory[Convert.ToInt32(instr.Arg1)] = _stack.Pop();
                            break;
                        case IROp.ADD:
                            {
                                var b = _stack.Pop();
                                var a = _stack.Pop();
                                _stack.Push(Add(a, b));
                            }
                            break;
                        case IROp.SUB:
                            {
                                var b = _stack.Pop();
                                var a = _stack.Pop();
                                _stack.Push(Sub(a, b));
                            }
                            break;
                        case IROp.MUL:
                            {
                                var b = _stack.Pop();
                                var a = _stack.Pop();
                                _stack.Push(Mul(a, b));
                            }
                            break;
                        case IROp.DIV:
                            {
                                var b = _stack.Pop();
                                var a = _stack.Pop();
                                _stack.Push(Div(a, b));
                            }
                            break;
                        case IROp.MOD:
                            {
                                var b = _stack.Pop();
                                var a = _stack.Pop();
                                _stack.Push(Mod(a, b));
                            }
                            break;
                        case IROp.CMP_EQ:
                            {
                                var b = _stack.Pop();
                                var a = _stack.Pop();
                                _stack.Push(Equals(a, b));
                            }
                            break;
                        case IROp.CMP_NEQ:
                            {
                                var b = _stack.Pop();
                                var a = _stack.Pop();
                                _stack.Push(!Equals(a, b));
                            }
                            break;
                        case IROp.CMP_LT:
                            {
                                var b = _stack.Pop();
                                var a = _stack.Pop();
                                _stack.Push(LessThan(a, b));
                            }
                            break;
                        case IROp.CMP_GT:
                            {
                                var b = _stack.Pop();
                                var a = _stack.Pop();
                                _stack.Push(GreaterThan(a, b));
                            }
                            break;
                        case IROp.CMP_LE:
                            {
                                var b = _stack.Pop();
                                var a = _stack.Pop();
                                _stack.Push(LessThanOrEqual(a, b));
                            }
                            break;
                        case IROp.CMP_GE:
                            {
                                var b = _stack.Pop();
                                var a = _stack.Pop();
                                _stack.Push(GreaterThanOrEqual(a, b));
                            }
                            break;
                        case IROp.AND:
                            {
                                var b = Convert.ToBoolean(_stack.Pop());
                                var a = Convert.ToBoolean(_stack.Pop());
                                _stack.Push(a && b);
                            }
                            break;
                        case IROp.OR:
                            {
                                var b = Convert.ToBoolean(_stack.Pop());
                                var a = Convert.ToBoolean(_stack.Pop());
                                _stack.Push(a || b);
                            }
                            break;
                        case IROp.NOT:
                            {
                                var a = Convert.ToBoolean(_stack.Pop());
                                _stack.Push(!a);
                            }
                            break;
                        case IROp.JUMP:
                            ip = _labels[(string)instr.Arg1];
                            break;
                        case IROp.JUMP_IF_FALSE:
                            {
                                var cond = Convert.ToBoolean(_stack.Pop());
                                if (!cond)
                                    ip = _labels[(string)instr.Arg1];
                            }
                            break;
                        case IROp.LABEL:
                            // No action needed at runtime
                            break;
                        case IROp.PRINT:
                            Console.WriteLine(_stack.Pop());
                            break;
                    }
                }
            }

            private object Add(object a, object b)
            {
                if (a is float || b is float)
                    return Convert.ToSingle(a) + Convert.ToSingle(b);
                return Convert.ToInt32(a) + Convert.ToInt32(b);
            }

            private object Sub(object a, object b)
            {
                if (a is float || b is float)
                    return Convert.ToSingle(a) - Convert.ToSingle(b);
                return Convert.ToInt32(a) - Convert.ToInt32(b);
            }

            private object Mul(object a, object b)
            {
                if (a is float || b is float)
                    return Convert.ToSingle(a) * Convert.ToSingle(b);
                return Convert.ToInt32(a) * Convert.ToInt32(b);
            }

            private object Div(object a, object b)
            {
                if (a is float || b is float)
                    return Convert.ToSingle(a) / Convert.ToSingle(b);
                return Convert.ToInt32(a) / Convert.ToInt32(b);
            }

            private object Mod(object a, object b)
            {
                if (a is float || b is float)
                    return Convert.ToSingle(a) % Convert.ToSingle(b);
                return Convert.ToInt32(a) % Convert.ToInt32(b);
            }

            private bool LessThan(object a, object b)
            {
                if (a is float || b is float)
                    return Convert.ToSingle(a) < Convert.ToSingle(b);
                return Convert.ToInt32(a) < Convert.ToInt32(b);
            }

            private bool GreaterThan(object a, object b)
            {
                if (a is float || b is float)
                    return Convert.ToSingle(a) > Convert.ToSingle(b);
                return Convert.ToInt32(a) > Convert.ToInt32(b);
            }

            private bool LessThanOrEqual(object a, object b)
            {
                if (a is float || b is float)
                    return Convert.ToSingle(a) <= Convert.ToSingle(b);
                return Convert.ToInt32(a) <= Convert.ToInt32(b);
            }

            private bool GreaterThanOrEqual(object a, object b)
            {
                if (a is float || b is float)
                    return Convert.ToSingle(a) >= Convert.ToSingle(b);
                return Convert.ToInt32(a) >= Convert.ToInt32(b);
            }
        }
    }
}
