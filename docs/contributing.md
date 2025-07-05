# Contributing Guide

Thank you for your interest in contributing to the UAV Agricultural Sensing project! This guide will help you get started with contributing code, documentation, examples, and improvements.

## 🚀 Getting Started

### Prerequisites

Before contributing, ensure you have:

- ✅ Completed the [Installation Guide](installation.md)
- ✅ Successfully run the [Quick Start Tutorial](tutorials/quickstart.md)
- ✅ Familiarity with ROS 2 and Python development
- ✅ Basic understanding of UAV systems and agriculture applications

### Development Environment Setup

1. **Fork the repository** on GitHub
2. **Clone your fork** locally:
```bash
git clone https://github.com/your-username/uav-agricultural-sensing.git
cd uav-agricultural-sensing
```

3. **Set up development environment**:
```bash
# Install development dependencies
pip3 install pre-commit pytest pytest-cov flake8 black

# Set up pre-commit hooks
pre-commit install

# Create development build
cd ~/ros2_ws
colcon build --packages-select uav_planning --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

4. **Configure git**:
```bash
git remote add upstream https://github.com/original-repo/uav-agricultural-sensing.git
git config --global user.name "Your Name"
git config --global user.email "your.email@example.com"
```

## 📝 How to Contribute

### Types of Contributions

We welcome several types of contributions:

- 🐛 **Bug Reports**: Issues with existing functionality
- ✨ **Feature Requests**: New capabilities or improvements
- 📚 **Documentation**: Improvements to guides, examples, and API docs
- 🧪 **Tests**: Additional test coverage and validation
- 🔧 **Code**: Bug fixes, new features, optimizations
- 📊 **Examples**: New configuration examples and use cases

### Contribution Process

1. **Create an issue** (for non-trivial changes)
2. **Fork and branch** from `main`
3. **Make your changes** following our coding standards
4. **Test your changes** thoroughly
5. **Update documentation** as needed
6. **Submit a pull request**

## 🐛 Reporting Bugs

### Before Reporting

1. **Search existing issues** to avoid duplicates
2. **Try the latest version** from the main branch
3. **Check the troubleshooting guide** for known issues

### Bug Report Template

Create an issue with this information:

```markdown
## Bug Description
Brief description of the bug

## Steps to Reproduce
1. Run command: `ros2 launch...`
2. Observe behavior: ...
3. Expected vs actual result

## Environment
- OS: Ubuntu 22.04
- ROS 2 version: Humble
- Package version: v1.0.0
- Hardware: Simulated/Real UAV

## Additional Context
- Error messages (full output)
- Configuration files used
- Screenshots/videos if helpful
```

## ✨ Requesting Features

### Feature Request Template

```markdown
## Feature Description
What new capability would you like to see?

## Use Case
What problem does this solve? Who would benefit?

## Proposed Implementation
If you have ideas about how to implement this

## Alternatives Considered
Other approaches you've thought about

## Additional Context
Any other information that would be helpful
```

## 💻 Code Contributions

### Coding Standards

#### Python Code Style

We follow PEP 8 with some specific guidelines:

```python
# Good: Clear, descriptive names
def generate_next_waypoint(self) -> Tuple[float, float, float]:
    """Generate the next waypoint using Lévy flight pattern."""
    pass

# Good: Type hints for all functions
def update_position(self, x: float, y: float, z: float) -> None:
    """Update current position."""
    pass

# Good: Docstrings for all public methods
class ButterflyExplorer:
    """
    Bio-inspired path planning using Lévy flight patterns.
    
    This class implements butterfly foraging behavior for efficient
    area coverage in agricultural applications.
    """
```

#### ROS 2 Specific Guidelines

```python
# Good: Use ROS 2 logging
self.get_logger().info("Waypoint generated successfully")

# Good: Proper QoS configuration
qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)

# Good: Parameter validation
if alpha <= 1.0 or alpha > 3.0:
    self.get_logger().error(f"Invalid alpha parameter: {alpha}")
    return
```

### Testing Requirements

All code contributions must include tests:

#### Unit Tests

```python
import unittest
from uav_planning.butterfly import ButterflyExplorer

class TestButterflyExplorer(unittest.TestCase):
    
    def setUp(self):
        self.explorer = ButterflyExplorer(
            x_min=-5, x_max=5,
            y_min=-5, y_max=5, 
            z_min=1, z_max=3
        )
    
    def test_waypoint_within_boundaries(self):
        """Test that generated waypoints respect boundaries."""
        for _ in range(100):
            x, y, z = self.explorer.generate_next_waypoint()
            self.assertGreaterEqual(x, -5)
            self.assertLessEqual(x, 5)
            # ... more assertions
```

#### Integration Tests

```python
import pytest
import rclpy
from uav_planning.bioinspired_path_generator import BioinspiredPathGenerator

def test_node_initialization():
    """Test that node initializes correctly."""
    rclpy.init()
    try:
        node = BioinspiredPathGenerator()
        assert node.get_name() == 'bioinspired_path_generator'
    finally:
        rclpy.shutdown()
```

#### Running Tests

```bash
# Run all tests
python -m pytest src/uav_planning/test/

# Run with coverage
python -m pytest --cov=uav_planning src/uav_planning/test/

# Run specific test
python -m pytest src/uav_planning/test/test_butterfly_generator.py::TestButterflyExplorer::test_waypoint_generation
```

### Documentation Requirements

All code must be documented:

- **Public APIs**: Complete docstrings with parameters, return values, and examples
- **Classes**: Class-level docstrings explaining purpose and usage
- **Complex algorithms**: Inline comments explaining non-obvious logic
- **Configuration**: Document all parameters and their effects

## 📚 Documentation Contributions

### Documentation Structure

Our documentation follows this structure:

```
docs/
├── README.md                 # Documentation index
├── installation.md           # Setup instructions
├── configuration.md          # Parameter reference
├── architecture.md           # System design
├── troubleshooting.md        # Common issues
├── tutorials/               # Step-by-step guides
│   └── quickstart.md
├── examples/                # Configuration examples
│   ├── README.md
│   └── *.yaml
└── api/                     # Code reference
    └── README.md
```

### Writing Guidelines

- **Clear and concise**: Use simple language
- **Actionable**: Include specific commands and examples
- **Up-to-date**: Test all commands and examples
- **Complete**: Cover edge cases and alternatives
- **Searchable**: Use descriptive headers and keywords

### Documentation Standards

```markdown
# Use clear hierarchical headers
## Main sections use H2
### Subsections use H3

# Include code blocks with syntax highlighting
```bash
ros2 launch uav_planning uav_simulation.launch.py
```

# Use tables for structured information
| Parameter | Type   | Default | Description           |
| --------- | ------ | ------- | --------------------- |
| alpha     | double | 1.5     | Lévy flight parameter |

# Include helpful tips
💡 **Tip**: Add this to your ~/.bashrc for automatic sourcing

# Mark important warnings
⚠️ **Warning**: This command will reset all parameters
```

## 🧪 Adding Examples

### Example Configuration Guidelines

When adding new example configurations:

1. **Clear use case**: Document the intended application
2. **Tested parameters**: Verify all parameters work together
3. **Performance notes**: Include expected flight time, coverage, etc.
4. **Hardware requirements**: Specify UAV platform needs
5. **Customization options**: Show how to modify for different scenarios

### Example Template

```yaml
# Example: [Use Case Name] Configuration
# 
# Description: Brief description of the use case and intended application
#
# Expected performance:
# - Coverage area: X acres
# - Flight time: Y minutes
# - Pattern characteristics: Z

/**:
  ros__parameters:
    # Boundary configuration
    x_min: 0.0
    x_max: 100.0
    # ... other parameters
    
    # Algorithm tuning for specific use case
    alpha: 1.5
    velocity: 8.0
    # ... other algorithm parameters
```

## 🔄 Pull Request Process

### Before Submitting

1. **Sync with upstream**:
```bash
git fetch upstream
git checkout main
git merge upstream/main
```

2. **Create feature branch**:
```bash
git checkout -b feature/your-feature-name
```

3. **Make changes and commit**:
```bash
git add .
git commit -m "feat: add new waypoint generation algorithm"
```

4. **Run tests**:
```bash
colcon build --packages-select uav_planning
colcon test --packages-select uav_planning
```

5. **Check code style**:
```bash
flake8 src/uav_planning/
black --check src/uav_planning/
```

### Pull Request Template

```markdown
## Description
Brief description of changes

## Type of Change
- [ ] Bug fix
- [ ] New feature
- [ ] Documentation update
- [ ] Performance improvement
- [ ] Code refactoring

## Testing
- [ ] Unit tests pass
- [ ] Integration tests pass
- [ ] Manual testing completed
- [ ] Documentation updated

## Checklist
- [ ] Code follows project style guidelines
- [ ] Self-review completed
- [ ] Tests added for new functionality
- [ ] Documentation updated
- [ ] No breaking changes (or documented)
```

### Review Process

1. **Automated checks**: CI/CD will run tests and style checks
2. **Code review**: Maintainers will review your code
3. **Feedback**: Address any requested changes
4. **Approval**: Once approved, your PR will be merged

## 🏷️ Commit Message Guidelines

We use conventional commits for clear commit history:

```bash
# Format: type(scope): description

# Types:
feat: new feature
fix: bug fix
docs: documentation changes
style: formatting changes
refactor: code refactoring
test: adding tests
chore: maintenance tasks

# Examples:
feat(algorithm): add adaptive alpha parameter adjustment
fix(ros2): resolve trajectory publishing timing issue
docs(api): update ButterflyExplorer class documentation
test(unit): add boundary constraint validation tests
```

## 🔐 Security Guidelines

When contributing:

- **No secrets**: Never commit passwords, API keys, or sensitive data
- **Input validation**: Validate all user inputs and parameters
- **Safe defaults**: Use conservative defaults for safety-critical parameters
- **Error handling**: Implement graceful error handling and recovery

## 📞 Getting Help

### Community Resources

- **GitHub Discussions**: For questions and general discussion
- **GitHub Issues**: For bug reports and feature requests
- **Email**: riccardo.enrico97@gmail.com for direct contact

### Development Questions

Before asking for help:

1. Search existing issues and discussions
2. Check the documentation thoroughly
3. Try debugging with increased logging
4. Prepare a minimal reproducible example

### Effective Questions

Include this information when asking for help:

```markdown
## What I'm trying to do
Clear description of your goal

## What I've tried
Steps you've already attempted

## Current behavior
What happens now

## Expected behavior
What you expected to happen

## Environment
- OS version
- ROS 2 version
- Package version
- Hardware setup

## Code/Configuration
Relevant code snippets or configuration files
```

## 🎉 Recognition

Contributors are recognized in several ways:

- **Contributors list**: Listed in README.md
- **Release notes**: Mentioned in version release notes
- **Documentation**: Author attribution in significant contributions
- **Community**: Recognition in community discussions

## 📄 License

By contributing, you agree that your contributions will be licensed under the same license as the project. Make sure you have the right to contribute the code you're submitting.

---

**Thank you for contributing to UAV Agricultural Sensing!** Your contributions help advance precision agriculture and autonomous systems research.

*Contributing guide last updated: January 5, 2025*
