# Setup Guide for GitHub Upload

## What Was Done

Your Django backend has been prepared for GitHub upload with proper security and sharing capabilities:

### 1. Security Improvements
- **Environment Variables**: Settings now use `.env` for sensitive data (SECRET_KEY, DEBUG)
- **Example Files**: Created `.env.example` and `ip_config.example.json` as templates
- **Updated .gitignore**: Ensures sensitive files (`.env`, `*.sqlite3`, `ip_config.json`) are NOT committed

### 2. Documentation
- **README.md**: Comprehensive setup instructions for new users
- **Includes**: Quick start, database setup, fixture loading, API docs, troubleshooting

### 3. Database Sharing
Your database structure will be shared through:
- **Fixtures**: `fixtures/curriculum_data.json` (12 units, 40+ levels, objectives)
- **Management Command**: `python manage.py load_curriculum`
- **Migrations**: Django migration files

### 4. Files Created/Updated

**Created:**
- `LAD/lad/.env.example` - Environment variables template
- `LAD/lad/README.md` - Complete setup documentation
- `config/ip_config.example.json` - Network configuration template

**Updated:**
- `LAD/lad/core/settings.py` - Now uses environment variables
- `LAD/lad/.gitignore` - Enhanced to exclude sensitive files
- `.gitignore` (root) - Protects database and config files

## How to Upload to GitHub

### Step 1: Create .env File (Don't Commit This!)

Before committing, create your local `.env` file:

```bash
cd LAD/lad
copy .env.example .env
```

Generate a new SECRET_KEY:

```bash
python -c "from django.core.management.utils import get_random_secret_key; print(get_random_secret_key())"
```

Add it to your `.env` file.

### Step 2: Verify What Will Be Committed

Check that sensitive files are ignored:

```bash
git status
```

**Should NOT see:**
- `.env` files
- `db.sqlite3`
- `config/ip_config.json`
- `__pycache__/` directories

**Should see:**
- `.env.example`
- `config/ip_config.example.json`
- `fixtures/*.json`
- `README.md`
- Migration files

### Step 3: Stage Django Backend Files

```bash
# From the root of your repo
git add LAD/
git add config/ip_config.example.json
git add SETUP_GUIDE.md
```

Or specifically:

```bash
git add LAD/lad/.env.example
git add LAD/lad/README.md
git add LAD/lad/.gitignore
git add LAD/lad/core/settings.py
git add LAD/lad/fixtures/
git add LAD/lad/requirements.txt
git add LAD/lad/manage.py
git add LAD/lad/apps/
git add LAD/lad/workspace/
git add config/ip_config.example.json
```

### Step 4: Commit Changes

```bash
git commit -m "Prepare Django backend for GitHub with database fixtures

- Add .env.example for environment configuration
- Add comprehensive README.md with setup instructions
- Update settings.py to use environment variables
- Add ip_config.example.json template
- Update .gitignore to protect sensitive data
- Include curriculum fixtures for database setup"
```

### Step 5: Push to GitHub

```bash
# If you haven't set up a remote yet
git remote add origin https://github.com/YOUR_USERNAME/YOUR_REPO.git

# Push to your branch
git push origin avanceNico

# Or push to main (if you want)
git push origin main
```

## For New Users Cloning Your Repo

When someone clones your repository, they'll follow these steps:

### 1. Clone Repository
```bash
git clone https://github.com/YOUR_USERNAME/YOUR_REPO.git
cd YOUR_REPO/LAD/lad
```

### 2. Set Up Environment
```bash
# Create virtual environment
python -m venv .venv
.venv\Scripts\activate  # Windows
source .venv/bin/activate  # Linux/Mac

# Install dependencies
pip install -r requirements.txt
```

### 3. Configure Environment
```bash
# Copy example files
copy .env.example .env  # Windows
cp .env.example .env  # Linux/Mac

# Edit .env and add a SECRET_KEY
# Generate one with:
# python -c "from django.core.management.utils import get_random_secret_key; print(get_random_secret_key())"
```

### 4. Set Up Database
```bash
# Run migrations
python manage.py migrate

# Load your curriculum data
python manage.py load_curriculum
```

### 5. Run Server
```bash
python manage.py runserver
```

## What They'll Get

Your users will get the **exact same database structure** with:
- All 12 curriculum units
- 40+ levels organized by units
- Learning objectives with points
- Complete data matching your setup

## Network Configuration (Optional)

If users need to configure network settings:

```bash
# From repo root
copy config\ip_config.example.json config\ip_config.json  # Windows
cp config/ip_config.example.json config/ip_config.json  # Linux/Mac

# Edit config/ip_config.json with their IP
```

## Security Notes

**NEVER commit:**
- `.env` files with actual SECRET_KEY
- `db.sqlite3` database files
- `config/ip_config.json` with real IPs

**ALWAYS commit:**
- `.env.example` (template only)
- `fixtures/*.json` (data structure)
- `config/ip_config.example.json` (template)
- Migration files
- `README.md` documentation

## Verification Checklist

Before pushing to GitHub, verify:

- [ ] `.env` is NOT in git status
- [ ] `db.sqlite3` is NOT in git status
- [ ] `config/ip_config.json` is NOT in git status
- [ ] `.env.example` IS in git status
- [ ] `config/ip_config.example.json` IS in git status
- [ ] `fixtures/curriculum_data.json` IS in git status
- [ ] `README.md` IS in git status
- [ ] Settings.py uses `os.getenv()` for SECRET_KEY

## Troubleshooting

### "Why is my .env showing in git status?"

```bash
# Make sure it's in .gitignore
git rm --cached .env
git commit -m "Remove .env from tracking"
```

### "Database file is too large"

Don't commit the database! Users will recreate it from fixtures.

### "SECRET_KEY is exposed"

If you accidentally committed a real SECRET_KEY:
1. Generate a new one
2. Update your local `.env`
3. Rewrite git history or rotate the key

## Next Steps

1. Review the README.md in `LAD/lad/README.md`
2. Test the setup process yourself by cloning to a new directory
3. Consider adding CI/CD pipelines
4. Add production deployment documentation
5. Set up GitHub Issues and Pull Request templates

## Support

Include this in your GitHub repo description:

```
🚀 L.A.D Platform - Learning Autonomous Driving

Django REST API backend with comprehensive curriculum management.

Quick Start:
1. Clone repo
2. Follow LAD/lad/README.md
3. Run `python manage.py load_curriculum`
4. Start learning!
```

---

**Your backend is now ready for GitHub! 🎉**

Users will be able to clone your repo and have the exact same database structure using the fixtures you've provided.
