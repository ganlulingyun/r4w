//! Simple Rust syntax highlighter for code display
//!
//! Uses egui's LayoutJob to create colored text spans.

use eframe::egui::{self, Color32, FontFamily, FontId, TextFormat};
use egui::text::LayoutJob;

/// Color scheme for Rust syntax highlighting (VS Code-like dark theme)
pub struct SyntaxColors;

impl SyntaxColors {
    /// Keywords: fn, let, mut, if, else, for, while, loop, match, return, pub, use, etc.
    pub const KEYWORD: Color32 = Color32::from_rgb(86, 156, 214); // Blue

    /// Types: f64, usize, Vec, Option, Result, Self, etc.
    pub const TYPE: Color32 = Color32::from_rgb(78, 201, 176); // Cyan/Teal

    /// Comments: // and /* */
    pub const COMMENT: Color32 = Color32::from_rgb(106, 153, 85); // Green

    /// Strings: "..." and '...'
    pub const STRING: Color32 = Color32::from_rgb(206, 145, 120); // Orange/Brown

    /// Numbers: 0, 1.0, 0x1F, 1e-10, etc.
    pub const NUMBER: Color32 = Color32::from_rgb(181, 206, 168); // Light Green

    /// Function calls and definitions
    pub const FUNCTION: Color32 = Color32::from_rgb(220, 220, 170); // Yellow

    /// Operators and punctuation
    pub const OPERATOR: Color32 = Color32::from_rgb(180, 180, 180); // Gray

    /// Default text color
    pub const DEFAULT: Color32 = Color32::from_rgb(212, 212, 212); // Light gray

    /// Macros like vec!, println!, etc.
    pub const MACRO: Color32 = Color32::from_rgb(78, 201, 176); // Same as type
}

/// Rust keywords to highlight
const KEYWORDS: &[&str] = &[
    "fn", "let", "mut", "const", "static", "if", "else", "match", "for", "while",
    "loop", "break", "continue", "return", "pub", "use", "mod", "struct", "enum",
    "impl", "trait", "where", "type", "as", "in", "ref", "self", "Self", "super",
    "crate", "move", "async", "await", "dyn", "true", "false",
];

/// Common Rust types to highlight
const TYPES: &[&str] = &[
    "f64", "f32", "i32", "i64", "u8", "u16", "u32", "u64", "usize", "isize",
    "bool", "char", "str", "String", "Vec", "Option", "Result", "Some", "None",
    "Ok", "Err", "Box", "Rc", "Arc", "HashMap", "HashSet", "IQSample", "Complex",
    "PI",
];

/// Create a syntax-highlighted LayoutJob from Rust code
pub fn highlight_rust_code(code: &str) -> LayoutJob {
    let mut job = LayoutJob::default();

    let font_id = FontId::new(13.0, FontFamily::Monospace);

    let default_format = TextFormat {
        font_id: font_id.clone(),
        color: SyntaxColors::DEFAULT,
        ..Default::default()
    };

    let mut chars = code.chars().peekable();
    let mut current_word = String::new();

    while let Some(c) = chars.next() {
        match c {
            // Comments
            '/' if chars.peek() == Some(&'/') => {
                // Flush current word
                flush_word(&mut job, &mut current_word, &font_id);

                // Consume the rest of the line as a comment
                let mut comment = String::from("//");
                chars.next(); // consume second /
                while let Some(&ch) = chars.peek() {
                    if ch == '\n' {
                        break;
                    }
                    comment.push(chars.next().unwrap());
                }
                job.append(
                    &comment,
                    0.0,
                    TextFormat {
                        font_id: font_id.clone(),
                        color: SyntaxColors::COMMENT,
                        ..Default::default()
                    },
                );
            }

            // Strings
            '"' => {
                flush_word(&mut job, &mut current_word, &font_id);

                let mut string = String::from("\"");
                let mut escaped = false;
                while let Some(ch) = chars.next() {
                    string.push(ch);
                    if escaped {
                        escaped = false;
                    } else if ch == '\\' {
                        escaped = true;
                    } else if ch == '"' {
                        break;
                    }
                }
                job.append(
                    &string,
                    0.0,
                    TextFormat {
                        font_id: font_id.clone(),
                        color: SyntaxColors::STRING,
                        ..Default::default()
                    },
                );
            }

            // Raw strings r#"..."#
            'r' if chars.peek() == Some(&'#') => {
                // Check if this is really a raw string
                let mut temp = String::from("r");
                let mut hash_count = 0;
                while chars.peek() == Some(&'#') {
                    temp.push(chars.next().unwrap());
                    hash_count += 1;
                }
                if chars.peek() == Some(&'"') {
                    temp.push(chars.next().unwrap());
                    // Read until closing "###
                    loop {
                        if let Some(ch) = chars.next() {
                            temp.push(ch);
                            if ch == '"' {
                                let mut closing_hashes = 0;
                                while closing_hashes < hash_count && chars.peek() == Some(&'#') {
                                    temp.push(chars.next().unwrap());
                                    closing_hashes += 1;
                                }
                                if closing_hashes == hash_count {
                                    break;
                                }
                            }
                        } else {
                            break;
                        }
                    }
                    flush_word(&mut job, &mut current_word, &font_id);
                    job.append(
                        &temp,
                        0.0,
                        TextFormat {
                            font_id: font_id.clone(),
                            color: SyntaxColors::STRING,
                            ..Default::default()
                        },
                    );
                } else {
                    // Not a raw string, treat as regular identifier
                    current_word.push_str(&temp);
                }
            }

            // Identifiers and keywords
            c if c.is_alphabetic() || c == '_' => {
                current_word.push(c);
            }

            // Numbers
            c if c.is_ascii_digit() => {
                if current_word.is_empty() || current_word.chars().all(|c| c.is_ascii_digit() || c == '.' || c == 'e' || c == 'E' || c == '-' || c == '_' || c == 'x' || c.is_ascii_hexdigit()) {
                    current_word.push(c);
                } else {
                    current_word.push(c);
                }
            }

            // Dots (could be part of number or method call)
            '.' => {
                if !current_word.is_empty() && current_word.chars().all(|c| c.is_ascii_digit() || c == '.') {
                    current_word.push('.');
                } else {
                    flush_word(&mut job, &mut current_word, &font_id);
                    job.append(".", 0.0, default_format.clone());
                }
            }

            // Operators and punctuation
            c if "(){}[]<>:;,=+-*/%&|!?@#$^~".contains(c) => {
                flush_word(&mut job, &mut current_word, &font_id);
                job.append(
                    &c.to_string(),
                    0.0,
                    TextFormat {
                        font_id: font_id.clone(),
                        color: SyntaxColors::OPERATOR,
                        ..Default::default()
                    },
                );
            }

            // Whitespace
            c if c.is_whitespace() => {
                flush_word(&mut job, &mut current_word, &font_id);
                job.append(&c.to_string(), 0.0, default_format.clone());
            }

            // Everything else
            _ => {
                flush_word(&mut job, &mut current_word, &font_id);
                job.append(&c.to_string(), 0.0, default_format.clone());
            }
        }
    }

    // Flush any remaining word
    flush_word(&mut job, &mut current_word, &font_id);

    job
}

/// Flush the current word to the job with appropriate coloring
fn flush_word(job: &mut LayoutJob, word: &mut String, font_id: &FontId) {
    if word.is_empty() {
        return;
    }

    let color = if KEYWORDS.contains(&word.as_str()) {
        SyntaxColors::KEYWORD
    } else if TYPES.contains(&word.as_str()) {
        SyntaxColors::TYPE
    } else if word.ends_with('!') {
        SyntaxColors::MACRO
    } else if word.chars().next().map(|c| c.is_ascii_digit()).unwrap_or(false)
        || (word.starts_with('-') && word.len() > 1 && word.chars().nth(1).map(|c| c.is_ascii_digit()).unwrap_or(false))
    {
        SyntaxColors::NUMBER
    } else {
        SyntaxColors::DEFAULT
    };

    job.append(
        word,
        0.0,
        TextFormat {
            font_id: font_id.clone(),
            color,
            ..Default::default()
        },
    );

    word.clear();
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_highlight_simple() {
        let code = "let x = 42;";
        let job = highlight_rust_code(code);
        assert!(!job.text.is_empty());
    }

    #[test]
    fn test_highlight_with_comment() {
        let code = "// This is a comment\nlet x = 1;";
        let job = highlight_rust_code(code);
        assert!(job.text.contains("comment"));
    }
}
