# Coding Style
When editing code, please use the coding style that you find in the file you are editing, or similar files around it.

# Operational details
### For Developers with Write Access
- Small changes limited to a single portion of the code base, can be pushed directly to master. Examples:
  - A 3 line bug fix for something trivial
  - Some typo fixes in the documentation
  - Bumping a library dependency to a new, but compatible newer version
- For larger changes, create a branch and pull request. Examples:
  - Bigger refactoring spanning over multiple files
  - Adding support for new hardware
  - Implementing a new network protocol for EFB communication

You can, of course, always create a PR if you want a second opinion on something


### For third-party developers
Please fork the repository, create a branch for what you want to do, push to that branch and then create a pull request.

If you are planning something bigger, feel free to contact us on Discord about your idea, or create a github discussion.


# Documentation
Code should, whenever possible, be self documenting and not require an external document.
External documentation only makes sense iff
- What you are documenting is something general, not related to a specific pice of code
- It requires a _lot_ of documentation
- It is intended for people that don't interact with the code

Examples:
- General setup of the operating system
- Setup of a dev environment
- Description/Documentation/Reference guide of a protocol



# Further Reading

### Development Environment setup
If you want to get started working on the code, see [dev_setup.md](https://github.com/stratux/stratux/docs/dev_setup.md)


### Interfacing with Stratux
If you are a developer of a third-party software, and want to interface with Stratux and use its data?
See [app-vendor-integration.md](https://github.com/stratux/stratux/docs/app-vendor-integration.md)

