# Git commands (command line)

The same work as [GIT_TUTORIAL.md](GIT_TUTORIAL.md), but typed instead of clicked. Use **Git Bash** (installed with Git) or PowerShell.

## 1. One-time setup
1. Install Git for Windows: https://git-scm.com/download/win (keep all the default options).
2. Tell Git who you are (use your GitHub name and email):
   ```
   git config --global user.name "Your Name"
   git config --global user.email "you@example.com"
   ```
3. Download the repo into a folder that is **not** in OneDrive:
   ```
   cd ~/Documents
   git clone https://github.com/zakaria0aladem/2DOF-Drone-Testbed.git
   cd 2DOF-Drone-Testbed
   ```
   The first time you push or pull, a window asks you to sign in to GitHub. Do it once and Git remembers.

## 2. Every time you work
Run these inside the repo folder:
```
git pull                         # 1. get the other person's latest changes
                                 # 2. edit files normally (CubeIDE, Python, ...)
git status                       # 3. see which files you changed
git add .                        # 4. pick all changes for the snapshot
git commit -m "Add ESC PWM"      # 5. save the snapshot with a short message
git push                         # 6. upload it to GitHub
```

## 3. Commands for looking around
| Command | What it shows |
|---|---|
| `git status` | changed files, and whether you are ahead or behind GitHub |
| `git diff` | the exact lines you changed (press `q` to quit) |
| `git log --oneline` | the list of past commits (press `q` to quit) |

## 4. Small undos
| Situation | Command |
|---|---|
| Throw away my edits to one file (not committed yet) | `git restore path/to/file` |
| I ran `git add` on a file by mistake | `git restore --staged path/to/file` |
| Fix the message of my last commit (before pushing) | `git commit --amend -m "Better message"` |

If you already pushed, don't try to undo it. Make a new commit that fixes it.

## 5. If pull or push complains
- **`push` is rejected ("fetch first")**: the other person pushed first. Run `git pull`, then `git push` again.
- **Conflict after `git pull`**: Git names the file. Open it, keep the right lines, delete the `<<<<<<<`, `=======` and `>>>>>>>` markers, then run:
  ```
  git add path/to/file
  git commit -m "Fix conflict"
  git push
  ```
- **You get stuck in a text editor screen** (Vim): type `:wq` and press Enter to save and leave.
