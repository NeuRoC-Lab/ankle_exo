# Welcome to the Ankle Exo documentation page

This webpage is meant for Si Qi and I to help reference and describe each other's work so that we are both on the same page. Documentation is also an important part of research !

The documentation is built on top of mkdocs, a static website generator, with the Read-the-Docs template.

For full documentation visit [mkdocs.org](https://www.mkdocs.org).

## Installation

* Mkdocs can be installed using python's package manager (pip) with `pip install mkdocs`. 
Note : you might have to create and activate a virtual environment (`python3 -m venv yourenv`) if the previous command returns an error related to `externally managed environments`

## Project layout

```yaml 
    mkdocs.yml    # The configuration file.
    docs/
        index.md  # The documentation homepage.
        ...       # Other markdown pages, images and other files.
```

The site map / blueprint is configured in the `mkdocs.yml` file, at the root folder of the mkdocs project (i.e `ankledocs`)

Whenever you want to make changes to the *pages* and their routing in the left column you need to modify the entries in  `mkdocs.yml`

For example let's consider this architecture : 
```yaml
nav:
- Home: index.md
- Electrical:
    - Overview: electrical/index.md
    [...]
- Software:
    - Overview: software/index.md
    [...]
- Mechanical:
    [...]
```

To add a new `Overview` page, after creating a `.md` file in the `mechanical` folder you need to link that file to the site map by adding an entry like so : 
```yaml
   - Overview: mechanical/index.md
```

## MKDOCS Commands

* `mkdocs serve` - Start the renderer. To view it open a tab at [127.0.0.1:8000](http://127.0.0.1:8000)

## Markdown syntax and conventions

Images should be placed in the top section folder relevant to the image (i.e, one of `electrical/images`, `software/images`, `mechanical/images`). Attachements can be placed in a `files` at the same level as `images`. 
To link an image to a markdown page file, use the syntax `![alias name](../images/yourimage.png)`. It will be rendered by on the webpage. 

For a downloadable file, the syntax is `[alias name](../files/datasheet.pdf)`. Note that there is no `!` preceding the `[`. 

Headers can be selected by order of font size (`#Title1`,`#Title2`, etc)

For more information about the markdown syntax checkout these resources : 

- [The markdown guide](https://www.markdownguide.org/)
- [Github markdown cheatsheet](https://enterprise.github.com/downloads/en/markdown-cheatsheet.pdf)

Enjoy !

