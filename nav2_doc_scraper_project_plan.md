# Nav2 Documentation Scraper - Project Plan

## Project Overview
Build a Python-based web scraper to extract the entire Nav2 documentation from docs.nav2.org and generate a comprehensive, searchable markdown file. The Nav2 documentation contains numerous configuration parameters across many packages, making it difficult to search and reference. This tool will consolidate everything into a single, well-structured document.

## Project Goals
1. Scrape all documentation pages from docs.nav2.org
2. Preserve document structure and hierarchy
3. Convert HTML to clean markdown format
4. Generate a single, searchable markdown file
5. Maintain proper cross-references and links
6. Handle nested documentation structures

## Technical Architecture

### Core Components

#### 1. Web Scraper Module (`scraper.py`)
**Purpose**: Crawl and download all pages from docs.nav2.org

**Key Features**:
- Start from the main index page (https://docs.nav2.org/configuration/index.html)
- Recursively follow all internal links
- Respect robots.txt and implement rate limiting
- Handle pagination and dynamic content
- Store raw HTML for processing

**Libraries**:
- `requests` - HTTP requests
- `beautifulsoup4` - HTML parsing
- `urllib.parse` - URL manipulation
- `time` - Rate limiting

**Key Functions**:
```python
def get_page(url: str) -> str
def extract_links(html: str, base_url: str) -> List[str]
def is_nav2_doc_url(url: str) -> bool
def crawl_documentation(start_url: str, max_depth: int = 10) -> Dict[str, str]
```

#### 2. Content Parser Module (`parser.py`)
**Purpose**: Extract meaningful content from HTML pages

**Key Features**:
- Remove navigation elements, headers, footers
- Extract main documentation content
- Preserve code blocks and formatting
- Extract metadata (title, section, subsection)
- Identify parameter tables and lists

**Libraries**:
- `beautifulsoup4` - HTML parsing
- `lxml` - Fast XML/HTML processing

**Key Functions**:
```python
def extract_title(soup: BeautifulSoup) -> str
def extract_content(soup: BeautifulSoup) -> BeautifulSoup
def extract_parameters(soup: BeautifulSoup) -> List[Parameter]
def clean_html(html: str) -> BeautifulSoup
```

#### 3. Markdown Generator Module (`markdown_generator.py`)
**Purpose**: Convert parsed HTML to clean markdown

**Key Features**:
- Convert HTML elements to markdown syntax
- Preserve tables, code blocks, lists
- Generate proper heading hierarchy
- Handle inline formatting (bold, italic, code)
- Create internal anchors for navigation

**Libraries**:
- `markdownify` - HTML to markdown conversion
- `re` - Regular expressions for cleanup

**Key Functions**:
```python
def html_to_markdown(html: str) -> str
def fix_heading_levels(markdown: str, base_level: int) -> str
def create_table_of_contents(sections: List[Section]) -> str
def sanitize_markdown(markdown: str) -> str
```

#### 4. Document Assembler Module (`assembler.py`)
**Purpose**: Combine all scraped content into a single structured document

**Key Features**:
- Organize content hierarchically
- Generate table of contents
- Create cross-references
- Add metadata and index
- Ensure consistent formatting

**Key Functions**:
```python
def organize_sections(pages: Dict[str, Page]) -> List[Section]
def generate_toc(sections: List[Section]) -> str
def assemble_document(sections: List[Section]) -> str
def add_metadata(document: str) -> str
```

## Data Models

```python
from dataclasses import dataclass
from typing import List, Optional

@dataclass
class Parameter:
    name: str
    type: str
    default: Optional[str]
    description: str

@dataclass
class Page:
    url: str
    title: str
    content: str
    parameters: List[Parameter]
    subsections: List[str]
    
@dataclass
class Section:
    title: str
    level: int
    content: str
    pages: List[Page]
    subsections: List['Section']
```

## Project Structure

```
nav2-doc-scraper/
├── README.md
├── requirements.txt
├── setup.py
├── .gitignore
├── config/
│   └── scraper_config.yaml
├── src/
│   └── nav2_scraper/
│       ├── __init__.py
│       ├── scraper.py
│       ├── parser.py
│       ├── markdown_generator.py
│       ├── assembler.py
│       └── utils.py
├── tests/
│   ├── test_scraper.py
│   ├── test_parser.py
│   ├── test_markdown_generator.py
│   └── test_assembler.py
├── data/
│   ├── raw/          # Raw HTML files
│   ├── processed/    # Intermediate processed data
│   └── output/       # Final markdown output
└── scripts/
    ├── run_scraper.py
    └── validate_output.py
```

## Implementation Phases

### Phase 1: Core Scraper (Week 1)
**Deliverables**:
- Functional web scraper that can crawl docs.nav2.org
- URL filtering and deduplication
- Rate limiting and error handling
- Save raw HTML files locally

**Tasks**:
1. Set up project structure
2. Implement basic HTTP request handling
3. Create URL queue management
4. Implement link extraction
5. Add rate limiting (1 request per second)
6. Handle HTTP errors and retries
7. Store raw HTML with metadata

### Phase 2: Content Parser (Week 2)
**Deliverables**:
- HTML content extraction
- Parameter table parsing
- Metadata extraction
- Clean, structured data model

**Tasks**:
1. Analyze Nav2 HTML structure
2. Implement content area identification
3. Create parameter table parser
4. Extract section hierarchy
5. Build data models
6. Implement content cleaning
7. Add validation checks

### Phase 3: Markdown Generation (Week 3)
**Deliverables**:
- HTML to markdown conversion
- Proper formatting preservation
- Code block handling
- Table conversion

**Tasks**:
1. Implement basic HTML to markdown conversion
2. Handle special cases (tables, code blocks)
3. Preserve internal links
4. Fix heading hierarchy
5. Clean up markdown formatting
6. Add syntax highlighting hints
7. Test on various content types

### Phase 4: Document Assembly (Week 4)
**Deliverables**:
- Single consolidated markdown file
- Table of contents
- Cross-references
- Search-friendly formatting

**Tasks**:
1. Design document hierarchy
2. Implement section organization
3. Generate table of contents
4. Create navigation anchors
5. Add document metadata
6. Implement cross-reference system
7. Final formatting and cleanup

### Phase 5: Testing & Polish (Week 5)
**Deliverables**:
- Comprehensive test suite
- Documentation
- Example output
- CLI tool

**Tasks**:
1. Write unit tests
2. Add integration tests
3. Create user documentation
4. Build CLI interface
5. Add progress indicators
6. Implement logging
7. Create example outputs

## Configuration File (`config/scraper_config.yaml`)

```yaml
scraper:
  start_url: "https://docs.nav2.org/configuration/index.html"
  base_domain: "docs.nav2.org"
  max_depth: 10
  rate_limit_seconds: 1.0
  timeout_seconds: 30
  max_retries: 3
  user_agent: "Nav2DocScraper/1.0"
  
parser:
  content_selector: "div.document"
  exclude_selectors:
    - "div.sphinxsidebar"
    - "div.related"
    - "div.footer"
  parameter_table_selector: "dl.py"
  
markdown:
  preserve_html_tables: true
  code_fence_style: "```"
  heading_style: "atx"  # Use # style headings
  
output:
  filename: "nav2_documentation_complete.md"
  include_toc: true
  include_metadata: true
  include_index: true
```

## Key Algorithms

### 1. Breadth-First Crawling
```python
def crawl_bfs(start_url: str, max_depth: int) -> Dict[str, str]:
    """
    Crawl documentation using breadth-first search
    """
    visited = set()
    queue = [(start_url, 0)]
    pages = {}
    
    while queue:
        url, depth = queue.pop(0)
        
        if url in visited or depth > max_depth:
            continue
            
        visited.add(url)
        html = get_page(url)
        pages[url] = html
        
        if depth < max_depth:
            links = extract_links(html, url)
            for link in links:
                if is_nav2_doc_url(link) and link not in visited:
                    queue.append((link, depth + 1))
    
    return pages
```

### 2. Hierarchical Document Organization
```python
def organize_hierarchy(pages: Dict[str, Page]) -> Section:
    """
    Organize pages into hierarchical sections based on URL structure
    """
    root = Section(title="Nav2 Documentation", level=1, content="", pages=[], subsections=[])
    
    for url, page in pages.items():
        path_parts = urlparse(url).path.strip('/').split('/')
        current_section = root
        
        for i, part in enumerate(path_parts[:-1]):
            # Find or create subsection
            subsection = find_subsection(current_section, part)
            if not subsection:
                subsection = Section(
                    title=part.replace('-', ' ').title(),
                    level=i + 2,
                    content="",
                    pages=[],
                    subsections=[]
                )
                current_section.subsections.append(subsection)
            current_section = subsection
        
        current_section.pages.append(page)
    
    return root
```

## Error Handling Strategy

1. **Network Errors**: Retry with exponential backoff (max 3 retries)
2. **Parsing Errors**: Log warning, skip problematic content, continue
3. **Missing Content**: Mark as incomplete, add to report
4. **Invalid URLs**: Log and skip
5. **Rate Limiting**: Respect HTTP 429, implement backoff

## Output Format

The final markdown file will have this structure:

```markdown
# Nav2 Complete Documentation

*Generated on: 2024-XX-XX*
*Source: https://docs.nav2.org*

## Table of Contents

1. [Configuration Guide](#configuration-guide)
   1.1. [Core Servers](#core-servers)
       1.1.1. [Behavior Server](#behavior-server)
       1.1.2. [Controller Server](#controller-server)
   ...

---

## Configuration Guide

[Content from configuration index]

### Core Servers

#### Behavior Server

[Full behavior server documentation]

##### Parameters

**parameter_name**
- Type: `string`
- Default: `value`
- Description: [description]

...
```

## Quality Assurance

### Validation Checks
1. All internal links are valid
2. No duplicate content
3. All parameters have descriptions
4. Code blocks are properly formatted
5. Tables are readable
6. Heading hierarchy is consistent

### Success Metrics
- [ ] 100% of documentation pages scraped
- [ ] All parameter tables captured
- [ ] Markdown passes linting
- [ ] File size < 50MB
- [ ] Search functionality works
- [ ] Cross-references are valid

## Dependencies

```txt
# requirements.txt
requests>=2.31.0
beautifulsoup4>=4.12.0
lxml>=5.1.0
markdownify>=0.11.0
pyyaml>=6.0
tqdm>=4.66.0
pytest>=7.4.0
black>=23.12.0
flake8>=7.0.0
```

## Usage Example

```bash
# Install dependencies
pip install -r requirements.txt

# Run the scraper
python scripts/run_scraper.py

# Output will be in data/output/nav2_documentation_complete.md

# Validate output
python scripts/validate_output.py data/output/nav2_documentation_complete.md
```

## Future Enhancements

1. **Version Support**: Scrape multiple Nav2 versions
2. **Incremental Updates**: Only scrape changed pages
3. **Search Index**: Generate searchable index file
4. **PDF Export**: Convert markdown to PDF
5. **Interactive CLI**: Add interactive selection of sections
6. **Diff Tool**: Compare different versions
7. **API Documentation**: Include C++/Python API docs

## Risk Mitigation

| Risk | Impact | Mitigation |
|------|--------|------------|
| Website structure changes | High | Use flexible CSS selectors, add tests |
| Rate limiting | Medium | Implement polite delays, respect robots.txt |
| Large document size | Medium | Add compression, split into sections option |
| Incomplete data | Low | Validation checks, manual review |
| Link rot | Low | Store timestamps, validate links |

## Timeline

**Total Duration**: 5 weeks

- Week 1: Core scraper
- Week 2: Content parser  
- Week 3: Markdown generation
- Week 4: Document assembly
- Week 5: Testing & polish

## Success Criteria

The project is successful when:
1. ✅ All Nav2 documentation pages are scraped
2. ✅ Single markdown file contains complete documentation
3. ✅ File is searchable with Ctrl+F
4. ✅ All parameters are documented with types and defaults
5. ✅ Code examples are preserved
6. ✅ Tables are properly formatted
7. ✅ Tool runs reliably without manual intervention

## Getting Started (For Cursor AI)

1. Create project directory structure
2. Set up virtual environment
3. Install dependencies
4. Implement scraper.py first (start with basic HTTP requests)
5. Test on a single page before scaling
6. Iterate through phases sequentially
7. Validate output after each phase

## Notes for Implementation

- Start with a small subset (e.g., just "Rotation Shim Controller" page) to test the pipeline
- Add verbose logging for debugging
- Use caching to avoid re-scraping during development
- Keep raw HTML files for debugging parser issues
- Test markdown rendering in a viewer before finalizing
- Consider edge cases: empty tables, nested lists, special characters
