var latex_macro = String.raw`\[
    \newcommand{\th}{\theta}
    \newcommand{\mc}[1]{\mathcal{#1}}
    \newcommand{\mv}[1]{\boldsymbol{#1}}
    \newcommand{\mvinfty}{\boldsymbol{\infty}}
    \newcommand{\mvzero}{\boldsymbol{0}}
    \newcommand{\x}{\mv{x}}
    \newcommand{\y}{\mv{y}}
    \newcommand{\dotx}{\dot{\mv{x}}}
    \newcommand{\q}{\mv{q}}
    \newcommand{\u}{\mv{u}}
    \newcommand{\K}{\mv{K}}
    \newcommand{\P}{\mv{P}}
    \newcommand{\rmP}{\mathrm{P}}
    \newcommand{\Q}{\mv{Q}}
    \newcommand{\R}{\mv{R}}
    \newcommand{\S}{\mv{S}}
    \newcommand{\dotq}{\dot{\mv{q}}}
    \newcommand{\ddotq}{\ddot{\mv{q}}}
    \newcommand{\f}[1]{\mathfrak{#1}}
    \newcommand{\bb}[1]{\mathbb{#1}}
    \newcommand{\d}[1]{\mathrm{d}#1}
    \newcommand{\ddt}{\frac{\d{}}{\d{t}}}
    \newcommand{\pLp}[1]{\frac{\partial L}{\partial #1}}
    \newcommand{\bm}[1]{\begin{bmatrix}#1\end{bmatrix}}
    \newcommand{\lagrangian}{F=\ddt\pLp{\dot{q}}-\pLp{q}}
    \newcommand{\mxi}{\mv{\xi}}
    \newcommand{\aligned}[1]{\begin{aligned}#1\end{aligned}}
\]`

document.getElementById('latex-macros').innerText = latex_macro;
