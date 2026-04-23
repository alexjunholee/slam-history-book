#!/usr/bin/env python3
"""
Build script: assemble chapter_*.md files into a single guide.html.
Adapted from sensor-fusion-guide/scripts/build_guide.py — Korean-only,
20-chapter structure (Ch.0-19) with 7 narrative parts.
Usage: python3 scripts/build_guide.py
"""

import os
import glob

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUTPUT = os.path.join(PROJECT_ROOT, "guide.html")


def read_chapters():
    pattern = os.path.join(PROJECT_ROOT, "chapter_*.md")
    files = sorted(glob.glob(pattern))
    parts = []
    for f in files:
        with open(f, "r", encoding="utf-8") as fh:
            parts.append(fh.read())
    return "\n\n---\n\n".join(parts)


OVERVIEW_CSS = """\
/* Guide Overview (at top of content) */
.guide-overview {
  margin: 0 0 80px 0;
  padding: 0 0 48px 0;
  border-bottom: 1px solid var(--border);
}
.guide-overview .overview-header { margin-bottom: 44px; }
.guide-overview .overview-title {
  font-family: 'SF Pro Display', -apple-system, BlinkMacSystemFont, 'Apple SD Gothic Neo', 'Pretendard', sans-serif;
  font-size: 48px;
  font-weight: 600;
  line-height: 1.08;
  letter-spacing: -0.018em;
  color: var(--text-heading);
  margin: 0 0 10px 0;
}
.guide-overview .overview-subtitle {
  font-size: 17px;
  color: var(--text-muted);
  letter-spacing: -0.014em;
  margin: 0;
}
.overview-group { margin-top: 36px; }
.overview-group:first-child { margin-top: 0; }
.overview-group-label {
  font-size: 11px;
  font-weight: 700;
  color: rgba(0, 0, 0, 0.42);
  text-transform: uppercase;
  letter-spacing: 0.08em;
  margin: 0 0 18px 0;
}
.overview-chapter { margin: 0 0 22px 0; }
.overview-chapter:last-child { margin-bottom: 0; }
.overview-chapter-link {
  font-family: 'SF Pro Display', -apple-system, BlinkMacSystemFont, 'Apple SD Gothic Neo', 'Pretendard', sans-serif;
  font-size: 19px;
  font-weight: 600;
  color: var(--text-heading);
  text-decoration: none;
  letter-spacing: -0.014em;
  display: inline-block;
}
.overview-chapter-link:hover { color: var(--accent); }
.overview-chapter-num {
  font-size: 12px;
  font-weight: 700;
  color: var(--accent);
  margin-right: 10px;
  letter-spacing: 0.02em;
  text-transform: uppercase;
}
.overview-sections {
  margin-top: 4px;
  font-size: 14px;
  line-height: 1.55;
  color: var(--text-muted);
  letter-spacing: -0.01em;
}
.overview-sections a {
  color: var(--text-muted);
  text-decoration: none;
}
.overview-sections a:hover { color: var(--accent); }
.overview-sections .sep {
  margin: 0 8px;
  color: var(--border-strong);
}
"""

# Narrative device boxes (📜 예언 vs 실제, 🔗 차용, 🧭 아직 열린 것)
# are authored as blockquotes in markdown; the default blockquote style
# covers them. No extra CSS needed — the emoji itself differentiates.


OVERVIEW_JS = r"""
  function buildOverview() {
    var chapterGroups = CHAPTER_GROUPS;

    var h1s = contentEl.querySelectorAll('h1');
    var chapters = {};

    h1s.forEach(function(h1) {
      var m = h1.textContent.trim().match(/(?:Chapter|Ch\.?)\s*(\d+)\s*[\u2014\u2013-]\s*(.+)/);
      if (!m) return;
      var chNum = parseInt(m[1], 10);
      if (!h1.id) h1.id = 'chapter-' + chNum;

      var sections = [];
      var node = h1.nextElementSibling;
      while (node && node.tagName !== 'H1') {
        if (node.tagName === 'H2') {
          if (!node.id) {
            node.id = node.textContent.toLowerCase()
              .replace(/[^\w\s가-힣-]/g, '').replace(/\s+/g, '-').substring(0, 60);
          }
          sections.push({ text: node.textContent.trim(), id: node.id });
        }
        node = node.nextElementSibling;
      }

      chapters[chNum] = { title: m[2].trim(), id: h1.id, sections: sections };
    });

    var total = Object.keys(chapters).length;
    if (total === 0) return;

    var overview = document.createElement('section');
    overview.className = 'guide-overview';

    var header = document.createElement('div');
    header.className = 'overview-header';
    var oTitle = document.createElement('h1');
    oTitle.className = 'overview-title';
    oTitle.textContent = OVERVIEW_TITLE;
    header.appendChild(oTitle);
    var oSub = document.createElement('p');
    oSub.className = 'overview-subtitle';
    oSub.textContent = 'SLAM · Spatial AI의 계보 추적 · 전 ' + total + '장';
    header.appendChild(oSub);
    overview.appendChild(header);

    chapterGroups.forEach(function(group) {
      var valid = group.chapters.filter(function(n) { return chapters[n]; });
      if (valid.length === 0) return;

      var groupEl = document.createElement('div');
      groupEl.className = 'overview-group';

      var label = document.createElement('div');
      label.className = 'overview-group-label';
      label.textContent = group.label;
      groupEl.appendChild(label);

      valid.forEach(function(chNum) {
        var ch = chapters[chNum];
        var chWrap = document.createElement('div');
        chWrap.className = 'overview-chapter';

        var chLink = document.createElement('a');
        chLink.className = 'overview-chapter-link';
        chLink.href = '#' + ch.id;
        var numSpan = document.createElement('span');
        numSpan.className = 'overview-chapter-num';
        numSpan.textContent = 'Ch.' + chNum;
        chLink.appendChild(numSpan);
        chLink.appendChild(document.createTextNode(ch.title));
        chWrap.appendChild(chLink);

        if (ch.sections.length) {
          var secsDiv = document.createElement('div');
          secsDiv.className = 'overview-sections';
          ch.sections.forEach(function(s, i) {
            if (i > 0) {
              var sep = document.createElement('span');
              sep.className = 'sep';
              sep.textContent = '·';
              secsDiv.appendChild(sep);
            }
            var sLink = document.createElement('a');
            sLink.href = '#' + s.id;
            sLink.textContent = s.text;
            secsDiv.appendChild(sLink);
          });
          chWrap.appendChild(secsDiv);
        }

        groupEl.appendChild(chWrap);
      });

      overview.appendChild(groupEl);
    });

    contentEl.insertBefore(overview, contentEl.firstChild);
  }
"""


def build_html(md_content):
    return f'''<!DOCTYPE html>
<html lang="ko">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>SLAM History Book — Spatial AI 계보 추적</title>
<link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/highlight.js/11.9.0/styles/atom-one-light.min.css">
<link rel="stylesheet" href="https://cdn.jsdelivr.net/npm/katex@0.16.10/dist/katex.min.css">
<link rel="preconnect" href="https://fonts.googleapis.com">
<link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=JetBrains+Mono:wght@400;500;600&display=swap">
<style>
*, *::before, *::after {{ box-sizing: border-box; margin: 0; padding: 0; }}

:root {{
  --bg: #ffffff;
  --bg-alt: #f5f5f7;
  --bg-sidebar: #f5f5f7;
  --bg-code: #f5f5f7;
  --bg-surface: #f5f5f7;

  --text: #1d1d1f;
  --text-body: rgba(0, 0, 0, 0.8);
  --text-muted: rgba(0, 0, 0, 0.56);
  --text-tertiary: rgba(0, 0, 0, 0.48);
  --text-heading: #1d1d1f;

  --accent: #0071e3;
  --link: #0066cc;

  --border: rgba(0, 0, 0, 0.08);
  --border-strong: rgba(0, 0, 0, 0.16);

  --sidebar-width: 300px;
  --topbar-height: 52px;
  --progress-height: 3px;

  --shadow-elev: rgba(0, 0, 0, 0.22) 3px 5px 30px 0px;
}}

html {{ -webkit-font-smoothing: antialiased; -moz-osx-font-smoothing: grayscale; }}

body {{
  font-family: 'SF Pro Text', -apple-system, BlinkMacSystemFont, 'Apple SD Gothic Neo', 'Pretendard', system-ui, 'Segoe UI', Roboto, 'Helvetica Neue', sans-serif;
  font-size: 17px;
  line-height: 1.47;
  letter-spacing: -0.022em;
  color: var(--text);
  background: var(--bg);
  overflow-x: hidden;
  font-feature-settings: "kern", "liga", "calt";
  word-break: keep-all;
  overflow-wrap: break-word;
}}

#progress-bar {{
  position: fixed;
  top: 0;
  left: 0;
  height: var(--progress-height);
  background: linear-gradient(90deg, #0071e3, #2997ff);
  z-index: 10000;
  transition: width 0.1s linear;
  width: 0%;
}}

#topbar {{
  position: fixed;
  top: var(--progress-height);
  left: 0;
  right: 0;
  height: var(--topbar-height);
  background: rgba(255, 255, 255, 0.72);
  backdrop-filter: saturate(180%) blur(20px);
  -webkit-backdrop-filter: saturate(180%) blur(20px);
  border-bottom: 1px solid var(--border);
  display: flex;
  align-items: center;
  padding: 0 24px;
  z-index: 1000;
}}

#topbar .menu-toggle {{
  display: none;
  background: none;
  border: none;
  color: var(--text);
  font-size: 22px;
  cursor: pointer;
  margin-right: 12px;
  padding: 6px 10px;
  border-radius: 8px;
}}

#topbar .menu-toggle:hover {{ background: rgba(0,0,0,0.04); }}

#topbar .title {{
  font-family: 'SF Pro Display', -apple-system, BlinkMacSystemFont, 'Apple SD Gothic Neo', sans-serif;
  font-size: 17px;
  font-weight: 600;
  color: var(--text-heading);
  letter-spacing: -0.022em;
  white-space: nowrap;
  overflow: hidden;
  text-overflow: ellipsis;
}}

#topbar .back-link {{
  margin-left: auto;
  color: var(--link);
  text-decoration: none;
  font-size: 13.5px;
  font-weight: 500;
  white-space: nowrap;
  padding: 7px 14px;
  border-radius: 980px;
  border: 1px solid var(--border);
  letter-spacing: -0.014em;
  transition: background 0.15s, border-color 0.15s, color 0.15s;
}}

#topbar .back-link:hover {{
  background: rgba(0, 113, 227, 0.06);
  border-color: var(--accent);
  color: var(--accent);
}}

#sidebar {{
  position: fixed;
  top: calc(var(--progress-height) + var(--topbar-height));
  left: 0;
  width: var(--sidebar-width);
  height: calc(100vh - var(--topbar-height) - var(--progress-height));
  background: var(--bg-sidebar);
  border-right: 1px solid var(--border);
  overflow-y: auto;
  z-index: 999;
  display: flex;
  flex-direction: column;
  scrollbar-width: thin;
  scrollbar-color: rgba(0,0,0,0.16) transparent;
}}

#sidebar::-webkit-scrollbar {{ width: 6px; }}
#sidebar::-webkit-scrollbar-track {{ background: transparent; }}
#sidebar::-webkit-scrollbar-thumb {{ background: rgba(0,0,0,0.16); border-radius: 3px; }}

#search-box {{
  padding: 14px 16px;
  border-bottom: 1px solid var(--border);
  background: rgba(255, 255, 255, 0.6);
  flex-shrink: 0;
}}

#search-input {{
  width: 100%;
  padding: 9px 12px;
  background: #ffffff;
  border: 1px solid var(--border);
  border-radius: 8px;
  color: var(--text);
  font-size: 13.5px;
  outline: none;
  font-family: inherit;
  letter-spacing: -0.016em;
  transition: border-color 0.15s, box-shadow 0.15s;
}}

#search-input:focus {{ border-color: var(--accent); box-shadow: 0 0 0 3px rgba(0, 113, 227, 0.15); }}
#search-input::placeholder {{ color: var(--text-muted); }}

#toc-container {{
  flex: 1;
  overflow-y: auto;
  padding: 8px 0 32px;
}}

#toc-container::-webkit-scrollbar {{ width: 4px; }}
#toc-container::-webkit-scrollbar-track {{ background: transparent; }}
#toc-container::-webkit-scrollbar-thumb {{ background: rgba(0,0,0,0.12); border-radius: 2px; }}

/* Chapter level (h1) */
.toc-item {{
  display: block;
  padding: 8px 16px 8px 20px;
  color: var(--text);
  text-decoration: none;
  font-size: 14px;
  font-weight: 600;
  line-height: 1.4;
  letter-spacing: -0.014em;
  border-left: 3px solid transparent;
  transition: color 0.15s, background 0.15s, border-color 0.15s;
  cursor: pointer;
}}

/* Section (h2) */
.toc-item.toc-h2 {{
  padding: 5px 16px 5px 36px;
  font-size: 13px;
  font-weight: 400;
  color: var(--text-muted);
  letter-spacing: -0.012em;
}}

.toc-item.toc-h2 + .toc-item:not(.toc-h2) {{ margin-top: 6px; }}

.toc-item:hover {{
  color: var(--text);
  background: rgba(0, 0, 0, 0.04);
}}

.toc-item.toc-h2:hover {{ color: var(--text); }}

.toc-item.active {{
  color: var(--accent);
  border-left-color: var(--accent);
  background: rgba(0, 113, 227, 0.08);
  font-weight: 700;
}}

.toc-item.toc-h2.active {{
  font-weight: 500;
  color: var(--accent);
}}

.toc-item.toc-hidden {{ display: none; }}

.toc-group-label {{
  display: block;
  padding: 22px 16px 6px 20px;
  font-size: 11px;
  font-weight: 700;
  color: rgba(0, 0, 0, 0.42);
  text-transform: uppercase;
  letter-spacing: 0.08em;
}}

.toc-group-label:first-child {{ padding-top: 8px; }}
.toc-group-label.toc-hidden {{ display: none; }}

.toc-no-results {{
  padding: 20px 16px;
  color: var(--text-muted);
  font-size: 13.5px;
  text-align: center;
  display: none;
  letter-spacing: -0.014em;
}}

#main-content {{
  margin-left: var(--sidebar-width);
  margin-top: calc(var(--progress-height) + var(--topbar-height));
  min-height: calc(100vh - var(--topbar-height) - var(--progress-height));
  padding: 56px 56px 96px;
}}

#content {{
  max-width: 820px;
  margin: 0 auto;
}}

#loading {{
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  padding: 80px 20px;
  color: var(--text-muted);
}}

.spinner {{
  width: 36px;
  height: 36px;
  border: 3px solid var(--border);
  border-top-color: var(--accent);
  border-radius: 50%;
  animation: spin 0.8s linear infinite;
  margin-bottom: 16px;
}}

@keyframes spin {{ to {{ transform: rotate(360deg); }} }}

#content h1, #content h2, #content h3, #content h4, #content h5, #content h6 {{
  font-family: 'SF Pro Display', -apple-system, BlinkMacSystemFont, 'Apple SD Gothic Neo', 'Pretendard', sans-serif;
  color: var(--text-heading);
  font-feature-settings: "ss01", "kern";
  scroll-margin-top: 72px;
}}

#content h1 {{
  font-size: 48px;
  font-weight: 600;
  line-height: 1.08;
  letter-spacing: -0.018em;
  margin: 0 0 28px 0;
  padding-bottom: 16px;
  border-bottom: 1px solid var(--border);
}}

#content h1:not(:first-of-type) {{
  margin-top: 96px;
}}

#content h1 + blockquote {{
  border-left: none;
  background: transparent;
  text-align: center;
  font-size: 18px;
  color: var(--text-muted);
  padding: 8px 0 24px;
  letter-spacing: -0.018em;
}}

#content h2 {{
  font-size: 30px;
  font-weight: 600;
  line-height: 1.10;
  letter-spacing: -0.014em;
  margin: 56px 0 18px 0;
  padding-bottom: 10px;
  border-bottom: 1px solid var(--border);
}}

#content h3 {{
  font-size: 22px;
  font-weight: 600;
  line-height: 1.16;
  letter-spacing: -0.012em;
  margin: 40px 0 12px 0;
}}

#content h4 {{
  font-size: 18px;
  font-weight: 600;
  line-height: 1.22;
  letter-spacing: -0.010em;
  margin: 28px 0 10px 0;
}}

#content h5, #content h6 {{
  font-size: 17px;
  font-weight: 600;
  line-height: 1.3;
  margin: 24px 0 8px 0;
}}

#content p {{ margin: 0 0 16px 0; color: var(--text-body); }}

#content a {{
  color: var(--link);
  text-decoration: none;
  transition: color 0.15s;
}}

#content a:hover {{ text-decoration: underline; text-underline-offset: 2px; }}

#content ul, #content ol {{
  margin: 0 0 16px 0;
  padding-left: 28px;
}}

#content li {{ margin-bottom: 6px; color: var(--text-body); }}
#content li > ul, #content li > ol {{ margin-top: 6px; margin-bottom: 6px; }}

#content hr {{
  border: none;
  border-top: 1px solid var(--border);
  margin: 56px 0;
}}

#content strong {{ color: var(--text); font-weight: 600; }}

#content img {{
  max-width: 100%;
  height: auto;
  border-radius: 12px;
  margin: 12px 0;
}}

#content code {{
  font-family: 'JetBrains Mono', ui-monospace, SFMono-Regular, Menlo, Consolas, 'Liberation Mono', monospace;
  font-size: 0.82em;
  letter-spacing: 0;
}}

#content :not(pre) > code {{
  background: var(--bg-code);
  padding: 2px 6px;
  border-radius: 5px;
  color: var(--text);
}}

#content pre {{
  background: var(--bg-code);
  border: none;
  border-radius: 12px;
  padding: 18px 20px;
  overflow-x: auto;
  margin: 0 0 20px 0;
  font-size: 13px;
  line-height: 1.55;
}}

#content pre code {{
  background: none;
  padding: 0;
  color: inherit;
  font-size: inherit;
}}

#content table {{
  width: 100%;
  border-collapse: collapse;
  margin: 0 0 20px 0;
  font-size: 15px;
  display: block;
  overflow-x: auto;
}}

#content thead {{ background: var(--bg-surface); }}

#content th {{
  padding: 12px 16px;
  text-align: left;
  color: var(--text-heading);
  font-weight: 600;
  font-size: 14px;
  border-bottom: 1px solid var(--border-strong);
  letter-spacing: -0.012em;
}}

#content td {{
  padding: 12px 16px;
  border-bottom: 1px solid var(--border);
  color: var(--text-body);
  letter-spacing: -0.016em;
}}

#content tbody tr:hover {{ background: rgba(0, 113, 227, 0.04); }}

#content blockquote {{
  border-left: 3px solid var(--accent);
  background: var(--bg-surface);
  padding: 14px 22px;
  margin: 0 0 20px 0;
  border-radius: 0 8px 8px 0;
  color: var(--text-body);
}}

#content blockquote p {{ margin-bottom: 8px; }}
#content blockquote p:last-child {{ margin-bottom: 0; }}

#content blockquote strong {{ color: var(--accent); }}

.katex {{ font-size: 1.05em; }}
.katex-display {{ overflow-x: auto; overflow-y: hidden; padding: 4px 0; margin: 20px 0; }}

::selection {{ background: rgba(0, 113, 227, 0.20); color: var(--text); }}

mark.search-highlight {{
  background: rgba(0, 113, 227, 0.18);
  color: inherit;
  padding: 1px 3px;
  border-radius: 3px;
}}

{OVERVIEW_CSS}

@media (max-width: 1024px) {{
  #main-content {{ padding-left: 40px; padding-right: 40px; }}
}}

@media (max-width: 900px) {{
  #topbar .menu-toggle {{ display: block; }}

  #sidebar {{
    transform: translateX(-100%);
    transition: transform 0.3s ease;
    box-shadow: none;
  }}

  #sidebar.open {{
    transform: translateX(0);
    box-shadow: var(--shadow-elev);
  }}

  #sidebar-overlay {{
    display: none;
    position: fixed;
    inset: 0;
    background: rgba(0,0,0,0.4);
    z-index: 998;
  }}

  #sidebar-overlay.visible {{ display: block; }}

  #main-content {{ margin-left: 0; padding: 24px 24px 56px; }}

  #content h1 {{ font-size: 36px; }}
  #content h2 {{ font-size: 26px; margin-top: 44px; }}
  #content h3 {{ font-size: 20px; }}
  #content h4 {{ font-size: 17px; }}
}}

@media (max-width: 600px) {{
  body {{ font-size: 16px; }}
  #content h1 {{ font-size: 30px; }}
  #content h2 {{ font-size: 22px; }}
  #content h3 {{ font-size: 18px; }}
  #main-content {{ padding: 20px 20px 48px; }}
}}

@media print {{
  #progress-bar, #topbar, #sidebar, #sidebar-overlay, #search-box, .menu-toggle {{ display: none !important; }}
  #main-content {{ margin: 0 !important; padding: 0 !important; }}
  #content {{ max-width: 100%; color: #000; }}
  #content h1, #content h2, #content h3, #content h4, #content strong {{ color: #000; }}
  #content a {{ color: #1a0dab; text-decoration: underline; }}
  #content pre, #content code {{ background: #f5f5f7; color: #000; border: none; }}
  #content blockquote {{ background: #f5f5f7; border-left-color: #999; }}
  #content table {{ border: 1px solid #ccc; }}
  #content th, #content td {{ border: 1px solid #ccc; }}
  body {{ background: #fff; }}
}}
</style>
</head>
<body>

<div id="progress-bar"></div>

<div id="topbar">
  <button class="menu-toggle" id="menu-toggle" aria-label="Toggle navigation">&#9776;</button>
  <span class="title">SLAM History Book</span>
  <a href="../" class="back-link">← repos</a>
</div>

<div id="sidebar-overlay"></div>

<nav id="sidebar">
  <div id="search-box">
    <input type="text" id="search-input" placeholder="챕터/섹션 검색... (Ctrl+K)">
  </div>
  <div id="toc-container"></div>
  <div class="toc-no-results" id="toc-no-results">검색 결과 없음</div>
</nav>

<main id="main-content">
  <div id="loading">
    <div class="spinner"></div>
    <span>가이드 렌더링 중...</span>
  </div>
  <div id="content" style="display:none;"></div>
</main>

<textarea id="md-source" style="display:none;">
{md_content}
</textarea>

<script src="https://cdnjs.cloudflare.com/ajax/libs/marked/12.0.1/marked.min.js"></script>
<script src="https://cdnjs.cloudflare.com/ajax/libs/highlight.js/11.9.0/highlight.min.js"></script>
<script src="https://cdn.jsdelivr.net/npm/katex@0.16.10/dist/katex.min.js"></script>
<script src="https://cdn.jsdelivr.net/npm/mermaid@10.9.1/dist/mermaid.min.js"></script>
<script>
(function() {{
  'use strict';

  var OVERVIEW_TITLE = 'SLAM History Book';

  // Ch.N → group label (used by both overview and sidebar TOC)
  var CHAPTER_GROUPS = [
    {{ label: '0 · 서문',        chapters: [0] }},
    {{ label: '1 · 선사시대',    chapters: [1, 2, 3] }},
    {{ label: '2 · 고전 SLAM',   chapters: [4, 5, 6] }},
    {{ label: '3 · 성숙기',      chapters: [7, 8, 9, 10] }},
    {{ label: '4 · 러닝 융합기', chapters: [11, 12, 13] }},
    {{ label: '5 · 표현의 혁명', chapters: [14, 15, 16] }},
    {{ label: '6 · 옆길과 결론', chapters: [17, 18, 19] }}
  ];

  // chapter number → group label (for TOC insertion when a new chapter starts a group)
  var GROUP_FIRST = {{}};
  CHAPTER_GROUPS.forEach(function(g) {{
    if (g.chapters.length) GROUP_FIRST[g.chapters[0]] = g.label;
  }});

  marked.setOptions({{
    highlight: function(code, lang) {{
      if (lang && hljs.getLanguage(lang)) {{
        try {{ return hljs.highlight(code, {{ language: lang }}).value; }} catch(e) {{}}
      }}
      try {{ return hljs.highlightAuto(code).value; }} catch(e) {{}}
      return code;
    }},
    breaks: false,
    gfm: true
  }});
  marked.use({{ renderer: {{ del: function(token) {{ return '~' + (token.text || token) + '~'; }} }} }});

  if (window.mermaid) {{
    mermaid.initialize({{ startOnLoad: false, theme: 'default', securityLevel: 'loose' }});
  }}

  function protectMath(src) {{
    var placeholders = [];
    var idx = 0;

    var codeBlocks = [];
    src = src.replace(/```[\s\S]*?```/g, function(m) {{
      codeBlocks.push(m);
      return '%%CODEBLOCK' + (codeBlocks.length - 1) + '%%';
    }});

    var inlineCodes = [];
    src = src.replace(/`[^`\\n]+`/g, function(m) {{
      inlineCodes.push(m);
      return '%%INLINECODE' + (inlineCodes.length - 1) + '%%';
    }});

    src = src.replace(/\$\$([\s\S]*?)\$\$/g, function(m, math) {{
      placeholders.push({{ display: true, math: math }});
      return '%%MATH' + (idx++) + '%%';
    }});

    src = src.replace(/\$([^\s$](?:[^$]*[^\s$])?)\$/g, function(m, math) {{
      placeholders.push({{ display: false, math: math }});
      return '%%MATH' + (idx++) + '%%';
    }});

    src = src.replace(/%%CODEBLOCK(\d+)%%/g, function(m, i) {{
      return codeBlocks[parseInt(i)];
    }});
    src = src.replace(/%%INLINECODE(\d+)%%/g, function(m, i) {{
      return inlineCodes[parseInt(i)];
    }});

    return {{ src: src, placeholders: placeholders }};
  }}

  function restoreAndRenderMath(html, placeholders) {{
    return html.replace(/%%MATH(\d+)%%/g, function(m, i) {{
      var p = placeholders[parseInt(i)];
      try {{
        return katex.renderToString(p.math, {{
          displayMode: p.display,
          throwOnError: false,
          trust: true
        }});
      }} catch(e) {{
        return p.display ? '$$' + p.math + '$$' : '$' + p.math + '$';
      }}
    }});
  }}

  var contentEl = document.getElementById('content');
  var loadingEl = document.getElementById('loading');
  var tocContainer = document.getElementById('toc-container');
  var state = {{ observer: null, scrollHandler: null }};

  function renderGuide() {{
    contentEl.style.display = 'none';
    loadingEl.style.display = 'flex';

    requestAnimationFrame(function() {{
      var src = document.getElementById('md-source').value;
      // CommonMark flank fix: **bold** followed by Korean sometimes fails to close.
      src = src.replace(/\*\*([^*\\n]+)\*\*(?=[가-힣])/g, '<strong>$1</strong>');
      var mathData = protectMath(src);
      var html = marked.parse(mathData.src);
      html = restoreAndRenderMath(html, mathData.placeholders);
      contentEl.innerHTML = html;

      if (window.mermaid) {{
        var mermaidBlocks = contentEl.querySelectorAll('pre > code.language-mermaid');
        mermaidBlocks.forEach(function(code, i) {{
          var pre = code.parentElement;
          var container = document.createElement('div');
          container.className = 'mermaid';
          container.id = 'mermaid-' + i;
          container.textContent = code.textContent;
          pre.replaceWith(container);
        }});
        try {{ mermaid.run({{ querySelector: '.mermaid' }}); }} catch(e) {{ console.error('mermaid', e); }}
      }}

      tocContainer.innerHTML = '';
      buildOverview();
      buildTOC();

      contentEl.style.display = 'block';
      loadingEl.style.display = 'none';
      setupScrollTracking();

      if (window.location.hash) {{
        var initId = window.location.hash.slice(1);
        var initTarget = document.getElementById(initId);
        if (initTarget) {{
          var initTop = initTarget.getBoundingClientRect().top + window.scrollY - 60;
          window.scrollTo(0, initTop);
        }}
      }}
    }});
  }}

  renderGuide();
  setupSearch();
  setupProgressBar();
  setupMobileMenu();
  setupKeyboard();
  setupTocDelegation();

  function setupTocDelegation() {{
    tocContainer.addEventListener('click', function(e) {{
      if (e.target.closest('.toc-item')) closeMobileSidebar();
    }});
  }}

{OVERVIEW_JS}

  // ---- Build Table of Contents (h1 chapters + h2 sections) ----
  function buildTOC() {{
    var headings = contentEl.querySelectorAll('h1, h2');
    var fragment = document.createDocumentFragment();

    headings.forEach(function(heading) {{
      if (!heading.id) {{
        heading.id = heading.textContent
          .toLowerCase()
          .replace(/[^\w\s가-힣-]/g, '')
          .replace(/\s+/g, '-')
          .substring(0, 60);
      }}

      if (heading.tagName === 'H1') {{
        var m = heading.textContent.trim().match(/Ch\.?\s*(\d+)/);
        if (m) {{
          var chNum = parseInt(m[1], 10);
          if (GROUP_FIRST[chNum]) {{
            var label = document.createElement('div');
            label.className = 'toc-group-label';
            label.textContent = GROUP_FIRST[chNum];
            fragment.appendChild(label);
          }}
        }}
      }}

      var link = document.createElement('a');
      link.className = 'toc-item toc-' + heading.tagName.toLowerCase();
      link.href = '#' + heading.id;
      link.textContent = heading.textContent;
      link.dataset.headingId = heading.id;
      fragment.appendChild(link);
    }});

    tocContainer.appendChild(fragment);
  }}

  // ---- Scroll tracking ----
  function setupScrollTracking() {{
    var headings = contentEl.querySelectorAll('h1, h2');
    var tocItems = tocContainer.querySelectorAll('.toc-item');
    if (headings.length === 0) return;

    var tocMap = {{}};
    tocItems.forEach(function(item) {{
      tocMap[item.dataset.headingId] = item;
    }});

    var currentActive = null;

    var observer = new IntersectionObserver(function(entries) {{
      var topHeading = null;
      var topOffset = Infinity;

      entries.forEach(function(entry) {{
        if (entry.isIntersecting) {{
          var rect = entry.target.getBoundingClientRect();
          if (rect.top < topOffset) {{
            topOffset = rect.top;
            topHeading = entry.target;
          }}
        }}
      }});

      if (topHeading) {{
        if (currentActive) currentActive.classList.remove('active');
        var item = tocMap[topHeading.id];
        if (item) {{
          item.classList.add('active');
          currentActive = item;
          var itemRect = item.getBoundingClientRect();
          var containerRect = tocContainer.getBoundingClientRect();
          if (itemRect.top < containerRect.top || itemRect.bottom > containerRect.bottom) {{
            item.scrollIntoView({{ block: 'center', behavior: 'auto' }});
          }}
        }}
      }}
    }}, {{
      rootMargin: '-60px 0px -70% 0px',
      threshold: 0
    }});

    headings.forEach(function(h) {{ observer.observe(h); }});
    state.observer = observer;

    // Fallback scroll handler for when no heading is inside the observer band
    if (!state.scrollHandler) {{
      var ticking = false;
      state.scrollHandler = function() {{
        if (!ticking) {{
          requestAnimationFrame(function() {{
            var curHeadings = contentEl.querySelectorAll('h1, h2');
            var curTocItems = tocContainer.querySelectorAll('.toc-item');
            var curMap = {{}};
            curTocItems.forEach(function(item) {{ curMap[item.dataset.headingId] = item; }});
            var found = null;
            var scrollTop = window.scrollY + 80;
            for (var i = curHeadings.length - 1; i >= 0; i--) {{
              if (curHeadings[i].offsetTop <= scrollTop) {{
                found = curHeadings[i];
                break;
              }}
            }}
            if (found) {{
              var active = tocContainer.querySelector('.toc-item.active');
              if (active) active.classList.remove('active');
              var item = curMap[found.id];
              if (item) item.classList.add('active');
            }}
            ticking = false;
          }});
          ticking = true;
        }}
      }};
      window.addEventListener('scroll', state.scrollHandler);
    }}
  }}

  function setupSearch() {{
    var searchInput = document.getElementById('search-input');
    var noResults = document.getElementById('toc-no-results');

    searchInput.addEventListener('input', function() {{
      var query = this.value.trim().toLowerCase();
      var tocItems = tocContainer.querySelectorAll('.toc-item');
      var groupLabels = tocContainer.querySelectorAll('.toc-group-label');

      if (!query) {{
        tocItems.forEach(function(item) {{ item.classList.remove('toc-hidden'); }});
        groupLabels.forEach(function(label) {{ label.classList.remove('toc-hidden'); }});
        noResults.style.display = 'none';
        return;
      }}

      var matchCount = 0;
      tocItems.forEach(function(item) {{
        var text = item.textContent.toLowerCase();
        if (text.includes(query)) {{
          item.classList.remove('toc-hidden');
          matchCount++;
        }} else {{
          item.classList.add('toc-hidden');
        }}
      }});

      groupLabels.forEach(function(label) {{ label.classList.add('toc-hidden'); }});
      noResults.style.display = matchCount === 0 ? 'block' : 'none';
    }});
  }}

  function setupProgressBar() {{
    var progressBar = document.getElementById('progress-bar');
    var ticking = false;

    function updateProgress() {{
      var scrollTop = window.scrollY;
      var docHeight = document.documentElement.scrollHeight - window.innerHeight;
      var progress = docHeight > 0 ? (scrollTop / docHeight) * 100 : 0;
      progressBar.style.width = Math.min(progress, 100) + '%';
    }}

    window.addEventListener('scroll', function() {{
      if (!ticking) {{
        requestAnimationFrame(function() {{
          updateProgress();
          ticking = false;
        }});
        ticking = true;
      }}
    }});

    updateProgress();
  }}

  function setupMobileMenu() {{
    var toggle = document.getElementById('menu-toggle');
    var sidebar = document.getElementById('sidebar');
    var overlay = document.querySelector('#sidebar-overlay');

    toggle.addEventListener('click', function() {{
      sidebar.classList.toggle('open');
      overlay.classList.toggle('visible');
    }});

    overlay.addEventListener('click', closeMobileSidebar);
  }}

  function closeMobileSidebar() {{
    document.getElementById('sidebar').classList.remove('open');
    document.querySelector('#sidebar-overlay').classList.remove('visible');
  }}

  function setupKeyboard() {{
    document.addEventListener('keydown', function(e) {{
      if ((e.ctrlKey || e.metaKey) && e.key === 'k') {{
        e.preventDefault();
        document.getElementById('search-input').focus();
      }}
    }});
  }}

}})();
</script>
</body>
</html>'''


def main():
    md = read_chapters()
    md = md.replace("</textarea>", "&lt;/textarea&gt;")
    html = build_html(md)
    with open(OUTPUT, "w", encoding="utf-8") as f:
        f.write(html)
    print(f"Built {OUTPUT}")
    print(f"  MD content: {len(md):,} chars")
    print(f"  HTML output: {len(html):,} chars")


if __name__ == "__main__":
    main()
