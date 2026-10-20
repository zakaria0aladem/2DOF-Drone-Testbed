# Git in 5 minutes (GitHub Desktop)

**Words you need:** a *repo* is the shared project folder that lives on GitHub. *Clone* = download it to your laptop once. *Commit* = save a snapshot of your changes with a short message. *Push* = send your commits to GitHub. *Pull* = get the other person's commits.

## 1. One-time setup
1. Install GitHub Desktop from https://desktop.github.com and open it.
2. Sign in: **File > Options > Accounts > Sign in** with your GitHub account.
3. Clone the repo: **File > Clone repository**, pick `2DOF-Drone-Testbed`, keep the suggested local path (`Documents\GitHub\...`), click **Clone**.
   Do not clone into a OneDrive folder: OneDrive syncing and Git fight over the same files.

## 2. Every time you work
1. **Pull first.** Click **Fetch origin**; if it turns into **Pull origin**, click it.
2. **Work normally.** Edit files in the cloned folder with CubeIDE, VS Code, Python, anything.
3. **Commit.** GitHub Desktop lists what you changed on the left. Type a short summary bottom-left (e.g. `Add ESC PWM on TIM8`) and click **Commit to main**.
4. **Push.** Click **Push origin** at the top. Now the other person can pull it.

Commit small and often, at least at the end of each session.

## 3. If there is a conflict
A conflict means you both changed the same lines. GitHub Desktop shows the file; open it, keep the right lines, delete the `<<<<<<<`, `=======`, `>>>>>>>` markers, save, then commit. If unsure, stop and ask the other person before pushing.

## 4. The one rule for working together
Each person works in their own folder (`firmware/` + `ai/piml/` = Zakaria, `ai/deepc/` = Amr), and **always pulls before starting**. Then conflicts almost never happen.

## Command-line equivalent (optional)
```
git clone https://github.com/zakaria0aladem/2DOF-Drone-Testbed.git   # once
git pull                        # 1. get the latest
git add .                       # 2. stage your changes
git commit -m "Short message"   # 3. snapshot
git push                        # 4. upload
```
