(TeX-add-style-hook
 "ProSlamSlides"
 (lambda ()
   (TeX-add-to-alist 'LaTeX-provided-class-options
                     '(("beamer" "16pt")))
   (TeX-run-style-hooks
    "latex2e"
    "beamer"
    "beamer10"))
 :latex)

