// SPDX-License-Identifier: GPL-3.0-or-later

//! Commit message check: Conventional Commits 1.0 with a footer allow-list.
//!
//! Rules (see CONTRIBUTING.md):
//! - header `type(scope)!: subject`, with a known type, an optional
//!   lower-case scope and an optional `!`;
//! - subject starts lower case, has no trailing period, and the header is
//!   at most 72 characters;
//! - a blank line separates header and body; body lines are at most 72
//!   characters unless they hold a word too long to wrap (URLs, paths);
//! - trailers are limited to `Fixes #N`, `Refs #N` and `BREAKING CHANGE: ...`.

use anyhow::{Context as _, bail};
use std::path::Path;
use std::process::Command;

const TYPES: [&str; 11] = [
    "feat", "fix", "perf", "refactor", "test", "docs", "build", "ci", "chore", "style", "revert",
];
const MAX_LINE: usize = 72;

/// Checks a commit message file, as written by `git commit` for the commit-msg hook.
pub fn check_file(path: &Path) -> anyhow::Result<()> {
    let text = std::fs::read_to_string(path)
        .with_context(|| format!("failed to read {}", path.display()))?;
    let problems = check_message(&strip_git_comments(&text));
    if problems.is_empty() {
        return Ok(());
    }
    bail!(
        "commit message rejected:\n  - {}\n\nSee CONTRIBUTING.md, \"Commit messages\".",
        problems.join("\n  - ")
    );
}

/// Checks every non-merge commit in a git revision range such as `origin/master..HEAD`.
pub fn check_range(range: &str) -> anyhow::Result<()> {
    let shas = git(&["rev-list", "--no-merges", "--reverse", range])?;
    let mut failures = Vec::new();
    let mut count = 0;
    for sha in shas.lines().filter(|l| !l.is_empty()) {
        count += 1;
        let message = git(&["log", "-1", "--format=%B", sha])?;
        let problems = check_message(&message);
        if !problems.is_empty() {
            let header = message.lines().next().unwrap_or_default();
            failures.push(format!(
                "{} {header}\n    - {}",
                &sha[..sha.len().min(10)],
                problems.join("\n    - ")
            ));
        }
    }
    if !failures.is_empty() {
        bail!(
            "{} of {count} commits have invalid messages:\n  {}\n\nSee CONTRIBUTING.md, \"Commit messages\".",
            failures.len(),
            failures.join("\n  ")
        );
    }
    eprintln!("commit check: {count} commits ok");
    Ok(())
}

fn git(args: &[&str]) -> anyhow::Result<String> {
    let output = Command::new("git")
        .args(args)
        .output()
        .context("failed to run git")?;
    if !output.status.success() {
        bail!(
            "git {} failed:\n{}",
            args.join(" "),
            String::from_utf8_lossy(&output.stderr)
        );
    }
    String::from_utf8(output.stdout).context("git printed invalid UTF-8")
}

/// Removes what git strips before committing: `#` comment lines and
/// everything below the scissors line of `git commit -v`.
fn strip_git_comments(text: &str) -> String {
    text.lines()
        .take_while(|l| !l.starts_with("# ------------------------ >8 ------------------------"))
        .filter(|l| !l.starts_with('#'))
        .collect::<Vec<_>>()
        .join("\n")
}

/// Returns every problem with `message`; empty if it is fine.
pub fn check_message(message: &str) -> Vec<String> {
    let lines: Vec<&str> = message.trim_end().lines().map(str::trim_end).collect();
    let mut problems = Vec::new();
    let Some(&header) = lines.first().filter(|h| !h.trim().is_empty()) else {
        return vec!["the message is empty".to_owned()];
    };

    // Messages git writes itself.
    if header.starts_with("Merge ") || header.starts_with("Revert \"") {
        return problems;
    }

    check_header(header, &mut problems);

    if lines.len() > 1 && !lines[1].is_empty() {
        problems.push("leave a blank line between the header and the body".to_owned());
    }

    for line in &lines[1..] {
        if looks_like_trailer(line) {
            if !is_allowed_trailer(line) {
                problems.push(format!(
                    "trailer `{line}` is not allowed; only `Fixes #N`, `Refs #N` and `BREAKING CHANGE: ...` are"
                ));
            }
        } else if line.chars().count() > MAX_LINE && !has_unbreakable_word(line) {
            problems.push(format!(
                "body line longer than {MAX_LINE} characters: `{line}`"
            ));
        }
    }
    problems
}

fn check_header(header: &str, problems: &mut Vec<String>) {
    let length = header.chars().count();
    if length > MAX_LINE {
        problems.push(format!(
            "header is {length} characters; keep it to {MAX_LINE} (aim for about 50)"
        ));
    }
    let Some((prefix, subject)) = header.split_once(": ") else {
        problems.push(format!(
            "header must look like `type(scope): subject`; got `{header}`"
        ));
        return;
    };

    let prefix = prefix.strip_suffix('!').unwrap_or(prefix);
    let (kind, scope) = match prefix.split_once('(') {
        Some((kind, rest)) => {
            let Some(scope) = rest.strip_suffix(')') else {
                problems.push(format!("unclosed scope in `{prefix}`"));
                return;
            };
            (kind, Some(scope))
        }
        None => (prefix, None),
    };
    if !TYPES.contains(&kind) {
        problems.push(format!(
            "unknown type `{kind}`; use one of {}",
            TYPES.join(", ")
        ));
    }
    if let Some(scope) = scope {
        let valid = !scope.is_empty()
            && scope
                .chars()
                .all(|c| c.is_ascii_lowercase() || c.is_ascii_digit() || c == '-');
        if !valid {
            problems.push(format!(
                "scope `{scope}` must be lower case letters, digits or hyphens"
            ));
        }
    }

    let subject_trimmed = subject.trim();
    if subject_trimmed.is_empty() {
        problems.push("subject is empty".to_owned());
        return;
    }
    if subject_trimmed != subject {
        problems.push("subject has leading or trailing spaces".to_owned());
    }
    if subject.chars().next().is_some_and(char::is_uppercase) {
        problems.push("subject must start with a lower-case letter".to_owned());
    }
    if subject.ends_with('.') {
        problems.push("subject must not end with a period".to_owned());
    }
}

/// Lines that act as git or Conventional Commits trailers: hyphenated keys
/// such as `Signed-off-by:`, `Token #value`, breaking-change notes and
/// issue keywords. Plain prose like `Note: ...` is not a trailer.
fn looks_like_trailer(line: &str) -> bool {
    const ISSUE_KEYWORDS: [&str; 5] = ["Fixes", "Refs", "Closes", "Resolves", "Fix"];
    let token_end = line
        .find(|c: char| !(c.is_ascii_alphanumeric() || c == '-'))
        .unwrap_or(line.len());
    let (token, rest) = line.split_at(token_end);
    if token.is_empty() || !token.starts_with(|c: char| c.is_ascii_alphabetic()) {
        return false;
    }
    let colon = rest.starts_with(": ") || rest == ":";
    let hash = rest.starts_with(" #");
    // Any `Token #value` line counts, so issue keywords only matter with a colon.
    line.starts_with("BREAKING CHANGE")
        || token == "BREAKING-CHANGE"
        || hash
        || (colon && (token.contains('-') || ISSUE_KEYWORDS.contains(&token)))
}

/// URLs and long paths can't be wrapped, so they may exceed the line limit.
fn has_unbreakable_word(line: &str) -> bool {
    line.split_whitespace()
        .any(|w| w.chars().count() > MAX_LINE / 2)
}

fn is_allowed_trailer(line: &str) -> bool {
    let issue = |rest: &str| !rest.is_empty() && rest.chars().all(|c| c.is_ascii_digit());
    if let Some(rest) = line.strip_prefix("Fixes #") {
        return issue(rest);
    }
    if let Some(rest) = line.strip_prefix("Refs #") {
        return issue(rest);
    }
    line.strip_prefix("BREAKING CHANGE: ")
        .is_some_and(|rest| !rest.trim().is_empty())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ok(message: &str) {
        let problems = check_message(message);
        assert!(problems.is_empty(), "{message:?}: {problems:?}");
    }

    fn bad(message: &str) {
        assert!(!check_message(message).is_empty(), "accepted {message:?}");
    }

    #[test]
    fn accepts_well_formed_messages() {
        ok("feat(geom): add knot insertion for NURBS curves");
        ok("fix(kernel): keep face orientation when merging coplanar fragments");
        ok("test(solver): cover conflicting tangent constraints");
        ok("chore: initialize repository");
        ok("feat(doc)!: drop format version 0");
        ok("feat(geom): add knot insertion for NURBS curves\n\n\
            Implements Boehm's algorithm (The NURBS Book, A5.1). Curve splitting\n\
            and Bézier decomposition both build on it.");
        ok("fix(io): close STL files on error\n\nThe writer leaked handles.\n\nFixes #12\nRefs #7");
        ok("feat(doc): store bodies as blobs\n\nBREAKING CHANGE: old files need migration");
        ok("Merge pull request #3 from someone/feat/x");
        ok("Revert \"feat(ui): add log panel\"");
    }

    #[test]
    fn rejects_bad_headers() {
        bad("");
        bad("added a thing");
        bad("feature: add a thing");
        bad("feat(Geom): add a thing");
        bad("feat(geom) add a thing");
        bad("feat(geom): Add a thing");
        bad("feat(geom): add a thing.");
        bad("feat(geom):  add a thing");
        bad("fixup! feat(geom): add a thing");
        bad(&format!("feat(geom): {}", "a".repeat(70)));
    }

    #[test]
    fn rejects_body_layout_problems() {
        bad("feat(geom): add a thing\nno blank line");
        bad(&format!(
            "feat(geom): add a thing\n\n{}",
            "word ".repeat(20)
        ));
        ok(&format!(
            "feat(geom): add a thing\n\nSee https://example.com/{}",
            "x".repeat(80)
        ));
    }

    #[test]
    fn rejects_trailers_outside_the_allow_list() {
        bad("feat(ui): add a panel\n\nCo-authored-by: Someone <a@b.c>");
        bad("feat(ui): add a panel\n\nSigned-off-by: Someone <a@b.c>");
        bad("feat(ui): add a panel\n\nBody text.\n\nFixes #12\nReviewed-by: X");
        bad("feat(ui): add a panel\n\nFixes: #12");
        bad("feat(ui): add a panel\n\nCo-authored-by: Someone <a@b.c>\n\nMore text after it.");
    }

    #[test]
    fn strips_git_comments_and_scissors() {
        let text = "feat(ui): add a panel\n# Please enter the commit message\n\
                    # ------------------------ >8 ------------------------\ndiff --git a b";
        assert_eq!(strip_git_comments(text), "feat(ui): add a panel");
    }
}
