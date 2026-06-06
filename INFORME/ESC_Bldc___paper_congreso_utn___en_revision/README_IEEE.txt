
HOW TO COMPILE (Overleaf / LaTeX):
1) Open the project and ensure the main document is 'main.tex' (Overleaf should pick it automatically).
   If not: Menu → Main document → select 'main.tex'.
2) Compiler: pdfLaTeX.
3) Build: LaTeX → BibTeX (if you have .bib) → LaTeX → LaTeX.
4) Your original content is in 'body_content.tex' (unchanged). Edit that file to update content.
5) Images paths are set via \graphicspath in 'main.tex'.
6) IEEEtran class is used: \documentclass[conference]{IEEEtran}.
