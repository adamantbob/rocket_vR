# Contributing to rocket_vR

Thank you for your interest in contributing to `rocket_vR`! We welcome contributions that improve the reliability and features of this flight control system.

## Getting Started

1.  **Fork the Repository**: Create a fork of the project on GitHub.
2.  **Clone Locally**:
    ```bash
    git clone https://github.com/your-username/rocket_vR.git
    cd rocket_vR
    ```
3.  **Setup Toolchain**: Follow the instructions in the [README.md](README.md) to install the necessary Rust targets and tools.

## Development Workflow

### Coding Standards
- Use `cargo fmt` to format your code before committing.
- Ensure your code is well-documented with `///` doc comments for public APIs.
- Avoid using `unsafe` unless absolutely necessary and well-documented.

### Running Tests
Most logic in `rocket-core` can be tested on your local machine:
```bash
cargo test -p rocket-core
```

### Submitting Changes
1.  **Create a Branch**: Use a descriptive name like `feature/new-sensor` or `fix/gps-lock`.
2.  **Commit Changes**: Provide clear and concise commit messages.
3.  **Open a Pull Request**: Explain the changes you've made and why they are beneficial.

## Reporting Issues
If you encounter a bug or have a feature request, please open an issue on GitHub. Be sure to include:
- A clear description of the problem.
- Steps to reproduce the issue.
- Logs or error messages (if applicable).

## Code of Conduct
Please be respectful and helpful to others in the community. We aim to foster a collaborative environment.
