#!/usr/bin/env bash
# Build the manuscript PDF.
# WSL2 + Windows MiKTeX, veya yerel texlive ile çalışır.

set -euo pipefail

cd "$(dirname "$0")"

# pdflatex'i bul: önce yerel, sonra Windows MiKTeX
if command -v pdflatex >/dev/null 2>&1; then
    PDFLATEX="pdflatex"
    BIBTEX="bibtex"
elif [[ -x "/mnt/c/Program Files/MiKTeX/miktex/bin/x64/pdflatex.exe" ]]; then
    PDFLATEX="/mnt/c/Program Files/MiKTeX/miktex/bin/x64/pdflatex.exe"
    BIBTEX="/mnt/c/Program Files/MiKTeX/miktex/bin/x64/bibtex.exe"
    echo "(Windows MiKTeX kullanılıyor)"
else
    echo "HATA: pdflatex bulunamadı."
    echo "Kurulum (Linux): sudo apt install texlive-latex-extra texlive-bibtex-extra texlive-publishers"
    echo "Veya Windows MiKTeX: https://miktex.org/download"
    exit 1
fi

echo "==> 1/4 pdflatex (ilk geçiş)"
"$PDFLATEX" -interaction=nonstopmode -halt-on-error main.tex >/dev/null

echo "==> 2/4 bibtex"
"$BIBTEX" main >/dev/null || echo "(bibtex uyarıları yok sayıldı)"

echo "==> 3/4 pdflatex (ikinci geçiş)"
"$PDFLATEX" -interaction=nonstopmode -halt-on-error main.tex >/dev/null

echo "==> 4/4 pdflatex (üçüncü geçiş)"
"$PDFLATEX" -interaction=nonstopmode -halt-on-error main.tex >/dev/null

echo ""
echo "Tamamlandı: $(pwd)/main.pdf"
ls -la main.pdf 2>/dev/null
