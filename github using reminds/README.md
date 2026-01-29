# Github Using Reminds

For a github new hand, the followings are commands I believe necessary for quick enters.

## Uploading files
```bash
git add .
git commit -m "具体内容"
git push origin main
```

## Repository setting fatal
It is possible to encounter the situation below when attempting to git push a repository recently set up:
```bash
git push -u origin main
Username for 'https://github.com': your_user_name
Password for 'https://your_user_name@github.com': just_type_your_login_password
remote: Invalid username or token. Password authentication is not supported for Git operations.
fatal: Authentication failed for 'https://github.com/your_user_name/your_repository_name.git/'
```
> REASON: Github doesn't allow login password authentication, so this way of pushing is invalid.

Possible solution: using the SSH way to enter:
```bash
git remote set-url origin git@github.com:your_user_name/your_repository_name.git
```

The fatal may be encountered also in pushing a branch and so forth, while using the SSH way is usually effective.