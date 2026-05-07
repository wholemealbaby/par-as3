#!/usr/bin/env python3
"""
Fetch issues from redbackbots-soccer repository and add them to the current repository.

This script:
1. Reads open issues from rmit-computing-technologies/redbackbots-soccer
2. Creates corresponding issues in the current repository
3. Adds the issues to GitHub Project ID 3

Configuration is loaded from (in priority order):
  1. Environment variables (GITHUB_TOKEN, GITHUB_REPOSITORY, PROJECT_ID, SOCCER_REPOSITORY)
  2. A .env file at .github/scripts/.env (or custom path via ENV_FILE env var)

Usage:
    GITHUB_TOKEN=<token> GITHUB_REPOSITORY=owner/repo python3 fetch_soccer_issues.py
    # or with .env file:
    python3 fetch_soccer_issues.py
"""

import os
import re
import sys
import json
import urllib.request
import urllib.error
import urllib.parse
from pathlib import Path

# ─── .env File Loader ────────────────────────────────────────────────────────


def load_env_file(env_path: str = ".github/scripts/.env") -> dict:
    """
    Load a .env file and return a dict of key-value pairs.
    Supports:
      - KEY=VALUE lines
      - # comments
      - Quoted values (single and double)
      - Trims whitespace
    """
    env_path = os.environ.get("ENV_FILE", env_path)
    env_file = Path(env_path)
    if not env_file.exists():
        return {}

    env_vars = {}
    for line in env_file.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        # Skip empty lines and comments
        if not line or line.startswith("#"):
            continue
        # Split on first =
        if "=" not in line:
            continue
        key, _, value = line.partition("=")
        key = key.strip()
        value = value.strip()
        # Strip surrounding quotes if present
        if len(value) >= 2 and value[0] == value[-1] and value[0] in ('"', "'"):
            value = value[1:-1]
        env_vars[key] = value
    return env_vars


# ─── Configuration ───────────────────────────────────────────────────────────

# Load .env file first (lowest priority)
_env_file = load_env_file()

# Environment variables override .env (higher priority)
GITHUB_TOKEN = os.environ.get("GITHUB_TOKEN") or _env_file.get("GITHUB_TOKEN")
GITHUB_REPOSITORY = os.environ.get("GITHUB_REPOSITORY") or _env_file.get("GITHUB_REPOSITORY", "")
PROJECT_ID = os.environ.get("PROJECT_ID") or _env_file.get("PROJECT_ID", "3")
SOCCER_REPOSITORY = os.environ.get("SOCCER_REPOSITORY") or _env_file.get("SOCCER_REPOSITORY", "rmit-computing-technologies/redbackbots-soccer")
ISSUES_DIR = Path(os.environ.get("ISSUES_DIR") or _env_file.get("ISSUES_DIR", ".issues"))

if not GITHUB_TOKEN:
    print("❌ GITHUB_TOKEN is required. Set it via environment variable or .env file.")
    sys.exit(1)

if not GITHUB_REPOSITORY:
    print("❌ GITHUB_REPOSITORY is required. Set it via environment variable or .env file.")
    sys.exit(1)

API_BASE = "https://api.github.com"
HEADERS = {
    "Authorization": f"Bearer {GITHUB_TOKEN}",
    "Accept": "application/vnd.github.v3+json",
    "Content-Type": "application/json",
}

# ─── HTTP Helpers ────────────────────────────────────────────────────────────


def api_request(method, url, data=None):
    """Make an authenticated GitHub API request."""
    if data is not None:
        data = json.dumps(data).encode("utf-8")
    req = urllib.request.Request(url, data=data, headers=HEADERS, method=method)
    try:
        with urllib.request.urlopen(req) as resp:
            body = resp.read().decode("utf-8")
            return json.loads(body) if body else {}
    except urllib.error.HTTPError as e:
        body = e.read().decode("utf-8")
        print(f"⚠️  HTTP {e.code} on {method} {url}: {body}")
        return None


def graphql_query(query, variables):
    """Make a GraphQL API request."""
    url = f"{API_BASE}/graphql"
    data = json.dumps({"query": query, "variables": variables}).encode("utf-8")
    req = urllib.request.Request(url, data=data, headers=HEADERS, method="POST")
    try:
        with urllib.request.urlopen(req) as resp:
            body = resp.read().decode("utf-8")
            result = json.loads(body)
            if "errors" in result:
                print(f"⚠️  GraphQL errors: {result['errors']}")
                return None
            return result
    except urllib.error.HTTPError as e:
        body = e.read().decode("utf-8")
        print(f"⚠️  HTTP {e.code} on GraphQL {url}: {body}")
        return None


def get_soccer_issues():
    """Fetch open issues from the soccer repository."""
    issues = []
    page = 1
    per_page = 100
    
    print(f"Fetching issues from {SOCCER_REPOSITORY}...")
    
    while True:
        url = f"{API_BASE}/repos/{SOCCER_REPOSITORY}/issues"
        params = {
            "state": "open",
            "per_page": per_page,
            "page": page
        }
        param_str = urllib.parse.urlencode(params)
        full_url = f"{url}?{param_str}"
        
        result = api_request("GET", full_url)
        if result is None:
            break
        
        if not result:
            break
        
        issues.extend(result)
        print(f"  Page {page}: fetched {len(result)} issues")
        
        if len(result) < per_page:
            break
        
        page += 1
    
    print(f"Total issues fetched: {len(issues)}")
    return issues


def get_existing_issues():
    """Get all existing issues in the current repository."""
    issues = []
    page = 1
    per_page = 100
    
    print(f"Fetching existing issues from {GITHUB_REPOSITORY}...")
    
    while True:
        url = f"{API_BASE}/repos/{GITHUB_REPOSITORY}/issues"
        params = {
            "state": "all",
            "per_page": per_page,
            "page": page
        }
        param_str = urllib.parse.urlencode(params)
        full_url = f"{url}?{param_str}"
        
        result = api_request("GET", full_url)
        if result is None:
            break
        
        if not result:
            break
        
        issues.extend(result)
        print(f"  Page {page}: fetched {len(result)} issues")
        
        if len(result) < per_page:
            break
        
        page += 1
    
    print(f"Total existing issues: {len(issues)}")
    return issues


def create_issue(title, body, labels=None, assignees=None):
    """Create a new issue in the current repository."""
    data = {
        "title": title,
        "body": body,
    }
    
    if labels:
        data["labels"] = labels
    
    if assignees:
        data["assignees"] = assignees
    
    url = f"{API_BASE}/repos/{GITHUB_REPOSITORY}/issues"
    result = api_request("POST", url, data)
    
    if result:
        print(f"  ✅ Created issue #{result['number']}: {result['title']}")
        return result
    return None


def add_issue_to_project(issue_number, project_id):
    """Add an issue to a GitHub Project."""
    # Get the issue node ID
    query = """
    query($owner: String!, $repo: String!, $issueNumber: Int!) {
        repository(owner: $owner, name: $repo) {
            issue(number: $issueNumber) {
                id
            }
        }
    }
    """
    
    owner, repo = GITHUB_REPOSITORY.split("/")
    
    node_result = graphql_query(query, {
        "owner": owner,
        "repo": repo,
        "issueNumber": issue_number
    })
    
    if not node_result or "data" not in node_result:
        print(f"  ⚠️  Could not get node ID for issue #{issue_number}")
        return False
    
    issue_id = node_result["data"]["repository"]["issue"]["id"]
    
    # Add to project
    mutation = """
    mutation($projectId: ID!, $contentId: ID!) {
        addProjectV2ItemById(input: {projectId: $projectId, contentId: $contentId}) {
            item {
                id
            }
        }
    }
    """
    
    result = graphql_query(mutation, {
        "projectId": project_id,
        "contentId": issue_id
    })
    
    if result and "data" in result:
        print(f"  ✅ Added issue #{issue_number} to project {project_id}")
        return True
    else:
        print(f"  ⚠️  Failed to add issue #{issue_number} to project {project_id}")
        return False


def get_label_color(label_name):
    """Get or create a label with a specific color."""
    # Check if label exists
    url = f"{API_BASE}/repos/{GITHUB_REPOSITORY}/labels/{label_name}"
    result = api_request("GET", url)
    
    if result and "name" in result:
        return result
    
    # Create label with default color
    labels = {
        "bug": "d73a4a",
        "enhancement": "a2eeef",
        "question": "d876e3",
        "documentation": "0075ca",
        "duplicate": "cfd3d7",
        "good first issue": "7057ff",
        "help wanted": "008672",
        "invalid": "e11d21",
        "wontfix": "ffffff",
        "priority-high": "ff0000",
        "priority-medium": "ffa500",
        "priority-low": "00ff00",
    }
    
    default_color = "aabbcc"
    color = labels.get(label_name.lower(), default_color)
    
    data = {
        "name": label_name,
        "color": color
    }
    
    url = f"{API_BASE}/repos/{GITHUB_REPOSITORY}/labels"
    result = api_request("POST", url, data)
    
    if result:
        print(f"  ℹ️  Created label '{label_name}'")
        return result
    
    return None


def sync_soccer_issues():
    """Main function to sync issues from soccer repository."""
    print("=" * 60)
    print("GitHub Issue Sync: redbackbots-soccer → Current Repository")
    print("=" * 60)
    print()
    
    # Labels to filter by (issues must have at least one of these labels)
    FILTER_LABELS = {"par", "par-as3"}
    
    # Fetch issues from soccer repository
    soccer_issues = get_soccer_issues()
    if not soccer_issues:
        print("No issues found in soccer repository.")
        return
    
    # Filter issues by labels
    filtered_issues = []
    for issue in soccer_issues:
        issue_labels = {label.get("name", "").lower() for label in issue.get("labels", [])}
        if FILTER_LABELS & issue_labels:  # Intersection - at least one common label
            filtered_issues.append(issue)
    
    print(f"Issues matching filter labels {FILTER_LABELS}: {len(filtered_issues)}")
    
    if not filtered_issues:
        print("No issues found matching the filter labels (par or par-as3).")
        return
    
    # Get existing issues to avoid duplicates
    existing_issues = get_existing_issues()
    
    # Create a lookup for existing issues by soccer issue URL
    existing_urls = {}
    for issue in existing_issues:
        html_url = issue.get("html_url", "")
        if "redbackbots-soccer" in html_url:
            # Extract the soccer issue number
            match = re.search(r'/redbackbots-soccer/issues/(\d+)', html_url)
            if match:
                existing_urls[match.group(1)] = issue
    
    # Process each soccer issue
    created_count = 0
    skipped_count = 0
    
    for soccer_issue in filtered_issues:
        issue_number = str(soccer_issue.get("number", ""))
        title = soccer_issue.get("title", "No title")
        body = soccer_issue.get("body", "")
        labels = soccer_issue.get("labels", [])
        assignees = soccer_issue.get("assignees", [])
        
        # Check if already exists
        if issue_number in existing_urls:
            print(f"⏭️  Skipping issue #{issue_number} (already exists)")
            skipped_count += 1
            continue
        
        # Build the body with reference to the original issue
        original_url = soccer_issue.get("html_url", "")
        original_author = soccer_issue.get("user", {}).get("login", "unknown")
        created_at = soccer_issue.get("created_at", "")
        
        reference = f"""
> **Original Issue:** [{issue_number}]({original_url})
> **Author:** @{original_author}
> **Created:** {created_at}
> **Source:** [{SOCCER_REPOSITORY}](https://github.com/{SOCCER_REPOSITORY})
"""
        
        full_body = f"{body}\n\n---\n\n{reference}"
        
        # Extract labels from soccer issue
        soccer_labels = [label.get("name", "") for label in labels]
        
        # Add soccer-specific labels with colors
        label_colors = {
            "bug": "d73a4a",
            "enhancement": "a2eeef",
            "question": "d876e3",
            "documentation": "0075ca",
            "duplicate": "cfd3d7",
            "good first issue": "7057ff",
            "help wanted": "008672",
            "invalid": "e11d21",
            "wontfix": "ffffff",
            "priority-high": "ff0000",
            "priority-medium": "ffa500",
            "priority-low": "00ff00",
            "soccer": "8844ff",
            "redbackbots": "ff4488",
        }
        
        # Create labels if they don't exist
        for label_name in soccer_labels:
            get_label_color(label_name)
        
        # Create the issue
        print(f"Creating issue #{issue_number}: {title}")
        new_issue = create_issue(
            title=f"[{SOCCER_REPOSITORY}#{issue_number}] {title}",
            body=full_body,
            labels=soccer_labels
        )
        
        if new_issue:
            created_count += 1
            # Add to project
            add_issue_to_project(new_issue["number"], PROJECT_ID)
    
    print()
    print("=" * 60)
    print(f"Sync Complete!")
    print(f"  Created: {created_count} issues")
    print(f"  Skipped: {skipped_count} issues (already exist)")
    print(f"  Total:   {len(soccer_issues)} issues processed")
    print("=" * 60)


if __name__ == "__main__":
    sync_soccer_issues()
