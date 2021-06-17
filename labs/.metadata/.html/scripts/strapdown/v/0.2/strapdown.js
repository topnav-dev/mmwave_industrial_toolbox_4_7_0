/**
 * marked - a markdown parser
 * Copyright (c) 2011-2014, Christopher Jeffrey. (MIT Licensed)
 * https://github.com/chjj/marked
 */
var chapters = '';
var lastChapterDepth = 0;
var codeBlockIdx = 0;

function striphtml(html)
{
   var tmp = document.createElement("DIV");
   tmp.innerHTML = html;
   return tmp.textContent || tmp.innerText || "";
}

;(function() {

/*

*/


/**
 * Block-Level Grammar
 */

var block = {
  newline: /^\n+/,
  code: /^(INVALIDATED_SHOULD_BE_SPACE{4}[^\n]+\n*)+/,
  fences: noop,
  hr: /^( *[-*_]){3,} *(?:\n+|$)/,
  heading: /^ *(#{1,6}) *([^\n]+?) *#* *(?:\n+|$)/,
  nptable: noop,
  lheading: /^([^\n]+)\n *(=|-){2,} *(?:\n+|$)/,
  blockquote: /^( *>[^\n]+(\n(?!def)[^\n]+)*\n*)+/,
  list: /^( *)(bull) [\s\S]+?(?:hr|def|\n{2,}(?! )(?!\1bull )\n*|\s*$)/,
  html: /^ *(?:comment *(?:\n|\s*$)|closed *(?:\n{2,}|\s*$)|closing *(?:\n{2,}|\s*$))/,
  def: /^ *\[([^\]]+)\]: *<?([^\s>]+)>?(?: +["(]([^\n]+)[")])? *(?:\n+|$)/,
  table: noop,
  paragraph: /^((?:[^\n]+\n?(?!hr|heading|lheading|blockquote|tag|def))+)\n*/,
  text: /^[^\n]+/,
  expandable: /^ *(\[\[\+[rgbyd]?)+([ \S]+)? *\n([\s\S]+?)\s*(\+\]\]|\1) *([^\n]*)/,
  callout: /^ *(\[{2,}[rgbyd]+?)(\!|[-a-z]+)?(>?) *([ \S]+)? *\n([\s\S]+?)\s*(\]\]|\1) *([^\n]*)/,
  alert: /^ *(\{{2,}[rgby]?)(>?)([\s\S]+?)\s*(\}\}|\1) *([^\n]*)/,
  quiz: /^ *(\[quiz\])([\s\S]+?)\s*(\1)/,
  quiz_multi: /^ *(\[quiz_multi\])([\s\S]+?)\s*(\1)/,
  tooltip: /\G *(\{{2,}\^([ rgbyd]?)?) ?([^\r\n\t]+?)?( *\n|\s*-->\s*)([\s\S]+?)\s*(\^\}\}|\1)/
};

block.bullet = /(?:[*+-]|\d+\.)/;
block.item = /^( *)(bull) [^\n]*(?:\n(?!\1bull )[^\n]*)*/;
block.item = replace(block.item, 'gm')
  (/bull/g, block.bullet)
  ();

block.list = replace(block.list)
  (/bull/g, block.bullet)
  ('hr', '\\n+(?=\\1?(?:[-*_] *){3,}(?:\\n+|$))')
  ('def', '\\n+(?=' + block.def.source + ')')
  ();

block.blockquote = replace(block.blockquote)
  ('def', block.def)
  ();

block._tag = '(?!(?:'
  + 'a|em|strong|small|s|cite|q|dfn|abbr|data|time|code'
  + '|var|samp|kbd|sub|sup|i|b|u|mark|ruby|rt|rp|bdi|bdo'
  + '|span|br|wbr|ins|del|img)\\b)\\w+(?!:/|[^\\w\\s@]*@)\\b';

block.html = replace(block.html)
  ('comment', /<!--[\s\S]*?-->/)
  ('closed', /<(tag)[\s\S]+?<\/\1>/)
  ('closing', /<tag(?:"[^"]*"|'[^']*'|[^'">])*?>/)
  (/tag/g, block._tag)
  ();

block.paragraph = replace(block.paragraph)
  ('hr', block.hr)
  ('heading', block.heading)
  ('lheading', block.lheading)
  ('blockquote', block.blockquote)
  ('tag', '<' + block._tag)
  ('def', block.def)
  ();

/**
 * Normal Block Grammar
 */

block.normal = merge({}, block);

/**
 * GFM Block Grammar
 */

block.gfm = merge({}, block.normal, {
  fences: /^ *(`{3,}|~{3,}) *(\S+)? *\n([\s\S]+?)\s*\1 *([^\n]*)/,
  paragraph: /^/
});

block.gfm.paragraph = replace(block.paragraph)
  ('(?!', '(?!'
    + block.gfm.fences.source.replace('\\1', '\\2') + '|'
    + block.list.source.replace('\\1', '\\3') + '|')
  ();

/**
 * GFM + Tables Block Grammar
 */

block.tables = merge({}, block.gfm, {
  nptable: /^ *(\S.*\|.*)\n *([-:]+ *\|[-| :]*)\n((?:.*\|.*(?:\n|$))*)\n*/,
  table: /^ *\|(.+)\n *\|( *[-:]+[-| :]*)\n((?: *\|.*(?:\n|$))*)\n*/
});

/**
 * Block Lexer
 */

function Lexer(options) {
  this.tokens = [];
  this.tokens.links = {};
  this.options = options || marked.defaults;
  this.rules = block.normal;

  if (this.options.gfm) {
    if (this.options.tables) {
      this.rules = block.tables;
    } else {
      this.rules = block.gfm;
    }
  }
}

/**
 * Expose Block Rules
 */

Lexer.rules = block;

/**
 * Static Lex Method
 */

Lexer.lex = function(src, options) {
  var lexer = new Lexer(options);
  return lexer.lex(src);
};

/**
 * Preprocessing
 */

Lexer.prototype.lex = function(src) {
  src = src
    .replace(/\r\n|\r/g, '\n')
    .replace(/\t/g, '    ')
    .replace(/\u00a0/g, ' ')
    .replace(/\u2424/g, '\n');

  return this.token(src, true);
};

/**
 * Lexing
 */

Lexer.prototype.token = function(src, top, bq) {
  var src = src.replace(/^ +$/gm, '')
    , next
    , loose
    , cap
    , bull
    , b
    , item
    , space
    , i
    , l;

  while (src) {
    // newline
    if (cap = this.rules.newline.exec(src)) {
      src = src.substring(cap[0].length);
      if (cap[0].length > 1) {
        this.tokens.push({
          type: 'space'
        });
      }
    }

    // code
    if (cap = this.rules.code.exec(src)) {
      src = src.substring(cap[0].length);
      cap = cap[0].replace(/^ {4}/gm, '');
      this.tokens.push({
        type: 'code_start',
        text: !this.options.pedantic
          ? cap.replace(/\n+$/, '')
          : cap
      });
      this.tokens.push({
        type: 'code_end'
      });
      continue;
    }

    // fences (gfm)
    if (cap = this.rules.fences.exec(src)) {
      src = src.substring(cap[0].length);
      this.tokens.push({
        type: 'code_start',
        lang: cap[2],
        text: cap[3]

      });
      this.token(cap[4], top, true);
      this.tokens.push({
        type: 'code_end'
      });
      continue;
    }

    // expandable
    if (cap = this.rules.expandable.exec(src)) {
      src = src.substring(cap[0].length);

      this.tokens.push({
        type: 'expandable_start',
        level: cap[1]
      });

    this.token(cap[2], top, true);

      this.tokens.push({
        type: 'expandable_title_end'
      });

    this.token(cap[3], top, true);

      this.tokens.push({
        type: 'expandable_end'
      });
      continue;
    }

    // callout
    if (cap = this.rules.callout.exec(src)) {
      src = src.substring(cap[0].length);

      this.tokens.push({
        type: 'callout_start',
        level: cap[1],
        symbol: cap[2],
        floating: cap[3]
      });

    this.token(cap[4], top, true);

      this.tokens.push({
        type: 'callout_title_end'
      });

    this.token(cap[5], top, true);

      this.tokens.push({
        type: 'callout_end'
      });
      continue;
    }

    // tooltip
    if (cap = this.rules.tooltip.exec(src)) {
      src = src.substring(cap[0].length);

      this.tokens.push({
        type: 'tooltip_start',
        color: cap[2]
      });

    this.token(cap[3], top, true);

      this.tokens.push({
        type: 'tooltip_title_end'
      });

    this.token(cap[5], top, true);

      this.tokens.push({
        type: 'tooltip_end'
      });
      continue;
    }

    // alert
    if (cap = this.rules.alert.exec(src)) {
      src = src.substring(cap[0].length);

      this.tokens.push({
        type: 'alert_start',
        level: cap[1],
        floating: cap[2]
      });

      this.token(cap[3], top, true);

      this.tokens.push({
        type: 'alert_end'
      });
      continue;
      }

    // quiz
    if (cap = this.rules.quiz.exec(src)) {
      src = src.substring(cap[0].length);

      this.tokens.push({
        type: 'quiz',
        body: cap[2]
      });
      continue;
    }

    // quiz-multi
    if (cap = this.rules.quiz_multi.exec(src)) {
      src = src.substring(cap[0].length);

      this.tokens.push({
        type: 'quiz_multi',
        body: cap[2]
      });
      continue;
    }

    // heading
    if (cap = this.rules.heading.exec(src)) {
      src = src.substring(cap[0].length);
      this.tokens.push({
        type: 'heading',
        depth: cap[1].length,
        text: cap[2]
      });
      continue;
    }

    // table no leading pipe (gfm)
    if (top && (cap = this.rules.nptable.exec(src))) {
      src = src.substring(cap[0].length);

      item = {
        type: 'table',
        header: cap[1].replace(/^ *| *\| *$/g, '').split(/ *\| */),
        align: cap[2].replace(/^ *|\| *$/g, '').split(/ *\| */),
        cells: cap[3].replace(/\n$/, '').split('\n')
      };

      for (i = 0; i < item.align.length; i++) {
        if (/^ *-+: *$/.test(item.align[i])) {
          item.align[i] = 'right';
        } else if (/^ *:-+: *$/.test(item.align[i])) {
          item.align[i] = 'center';
        } else if (/^ *:-+ *$/.test(item.align[i])) {
          item.align[i] = 'left';
        } else {
          item.align[i] = null;
        }
      }

      for (i = 0; i < item.cells.length; i++) {
        item.cells[i] = item.cells[i].split(/ *\| */);
      }

      this.tokens.push(item);

      continue;
    }

    // lheading
    if (cap = this.rules.lheading.exec(src)) {
      src = src.substring(cap[0].length);
      this.tokens.push({
        type: 'heading',
        depth: cap[2] === '=' ? 1 : 2,
        text: cap[1]
      });
      continue;
    }

    // hr
    if (cap = this.rules.hr.exec(src)) {
      src = src.substring(cap[0].length);
      this.tokens.push({
        type: 'hr'
      });
      continue;
    }

    // blockquote
    if (cap = this.rules.blockquote.exec(src)) {
      src = src.substring(cap[0].length);

      this.tokens.push({
        type: 'blockquote_start'
      });

      cap = cap[0].replace(/^ *> ?/gm, '');

      // Pass `top` to keep the current
      // "toplevel" state. This is exactly
      // how markdown.pl works.
      this.token(cap, top, true);

      this.tokens.push({
        type: 'blockquote_end'
      });

      continue;
    }

    // list
    if (cap = this.rules.list.exec(src)) {
      src = src.substring(cap[0].length);
      bull = cap[2];

      this.tokens.push({
        type: 'list_start',
        ordered: bull.length > 1
      });

      // Get each top-level item.
      cap = cap[0].match(this.rules.item);

      next = false;
      l = cap.length;
      i = 0;

      for (; i < l; i++) {
        item = cap[i];

        // Remove the list item's bullet
        // so it is seen as the next token.
        space = item.length;
        item = item.replace(/^ *([*+-]|\d+\.) +/, '');

        // Outdent whatever the
        // list item contains. Hacky.
        if (~item.indexOf('\n ')) {
          space -= item.length;
          item = !this.options.pedantic
            ? item.replace(new RegExp('^ {1,' + space + '}', 'gm'), '')
            : item.replace(/^ {1,4}/gm, '');
        }

        // Determine whether the next list item belongs here.
        // Backpedal if it does not belong in this list.
        if (this.options.smartLists && i !== l - 1) {
          b = block.bullet.exec(cap[i + 1])[0];
          if (bull !== b && !(bull.length > 1 && b.length > 1)) {
            src = cap.slice(i + 1).join('\n') + src;
            i = l - 1;
          }
        }

        // Determine whether item is loose or not.
        // Use: /(^|\n)(?! )[^\n]+\n\n(?!\s*$)/
        // for discount behavior.
        loose = next || /\n\n(?!\s*$)/.test(item);
        if (i !== l - 1) {
          next = item.charAt(item.length - 1) === '\n';
          if (!loose) loose = next;
        }

        this.tokens.push({
          type: loose
            ? 'loose_item_start'
            : 'list_item_start'
        });

        // Recurse.
        this.token(item, false, bq);

        this.tokens.push({
          type: 'list_item_end'
        });
      }

      this.tokens.push({
        type: 'list_end'
      });

      continue;
    }

    // html
    if (cap = this.rules.html.exec(src)) {
      src = src.substring(cap[0].length);
      this.tokens.push({
        type: this.options.sanitize
          ? 'paragraph'
          : 'html',
        pre: cap[1] === 'pre' || cap[1] === 'script' || cap[1] === 'style',
        text: cap[0]
      });
      continue;
    }

    // def
    if ((!bq && top) && (cap = this.rules.def.exec(src))) {
      src = src.substring(cap[0].length);
      this.tokens.links[cap[1].toLowerCase()] = {
        href: cap[2],
        title: cap[3]
      };
      continue;
    }

    // table (gfm)
    if (top && (cap = this.rules.table.exec(src))) {
      src = src.substring(cap[0].length);

      item = {
        type: 'table',
        header: cap[1].replace(/^ *| *\| *$/g, '').split(/ *\| */),
        align: cap[2].replace(/^ *|\| *$/g, '').split(/ *\| */),
        cells: cap[3].replace(/(?: *\| *)?\n$/, '').split('\n')
      };

      for (i = 0; i < item.align.length; i++) {
        if (/^ *-+: *$/.test(item.align[i])) {
          item.align[i] = 'right';
        } else if (/^ *:-+: *$/.test(item.align[i])) {
          item.align[i] = 'center';
        } else if (/^ *:-+ *$/.test(item.align[i])) {
          item.align[i] = 'left';
        } else {
          item.align[i] = null;
        }
      }

      for (i = 0; i < item.cells.length; i++) {
        item.cells[i] = item.cells[i]
          .replace(/^ *\| *| *\| *$/g, '')
          .split(/ *\| */);
      }

      this.tokens.push(item);

      continue;
    }

    // top-level paragraph
    if (top && (cap = this.rules.paragraph.exec(src))) {
      src = src.substring(cap[0].length);
      this.tokens.push({
        type: 'paragraph',
        text: cap[1].charAt(cap[1].length - 1) === '\n'
          ? cap[1].slice(0, -1)
          : cap[1]
      });
      continue;
    }

    // text
    if (cap = this.rules.text.exec(src)) {
      // Top-level should never reach here.
      src = src.substring(cap[0].length);
      this.tokens.push({
        type: 'text',
        text: cap[0]
      });
      continue;
    }

    if (src) {
      throw new
        Error('Infinite loop on byte: ' + src.charCodeAt(0));
    }
  }

  return this.tokens;
};

/**
 * Inline-Level Grammar
 */

var inline = {
  escape: /^\\([\\`*{}\[\]()#+\-.!_>])/,
  autolink: /^<([^ >]+(@|:\/)[^ >]+)>/,
  url: noop,
  tag: /^<!--[\s\S]*?-->|^<\/?\w+(?:"[^"]*"|'[^']*'|[^'">])*?>/,
  link: /^!?\[(inside)\]\(href\)/,
  inl_tooltip: /^!?\[(inside)\]\{([\s\S]+?)\}/,
  reflink: /^!?\[(inside)\]\s*\[([^\]]*)\]/,
  nolink: /^!?\[((?:\[[^\]]*\]|[^\[\]])*)\]/,
  strong: /^__([\s\S]+?)__(?!_)|^\*\*([\s\S]+?)\*\*(?!\*)/,
  em: /^\b_((?:__|[\s\S])+?)_\b|^\*((?:\*\*|[\s\S])+?)\*(?!\*)/,
  code: /^(`+)\s*([\s\S]*?[^`])\s*\1(?!`)/,
  br: /^ {2,}\n(?!\s*$)/,
  del: noop,
  text: /^[\s\S]+?(?=[\\<!\[{_*`]| {2,}\n|$)/
};

inline._inside = /(?:\[[^\]]*\]|[^\[\]]|\](?=[^\[]*\]))*/;
inline._href = /\s*<?([\s\S]*?)>?(?:\s+['"]([\s\S]*?)['"])?\s*/;

inline.link = replace(inline.link)
  ('inside', inline._inside)
  ('href', inline._href)
  ();

inline.inl_tooltip = replace(inline.inl_tooltip)
  ('inside', inline._inside)
  ();

inline.reflink = replace(inline.reflink)
  ('inside', inline._inside)
  ();

/**
 * Normal Inline Grammar
 */

inline.normal = merge({}, inline);

/**
 * Pedantic Inline Grammar
 */

inline.pedantic = merge({}, inline.normal, {
  strong: /^__(?=\S)([\s\S]*?\S)__(?!_)|^\*\*(?=\S)([\s\S]*?\S)\*\*(?!\*)/,
  em: /^_(?=\S)([\s\S]*?\S)_(?!_)|^\*(?=\S)([\s\S]*?\S)\*(?!\*)/
});

/**
 * GFM Inline Grammar
 */

inline.gfm = merge({}, inline.normal, {
  escape: replace(inline.escape)('])', '~|])')(),
  url: /^(https?:\/\/[^\s<]+[^<.,:;"')\]\s])/,
  del: /^~~(?=\S)([\s\S]*?\S)~~/,
  text: replace(inline.text)
    (']|', '~]|')
    ('|', '|https?://|')
    ()
});

/**
 * GFM + Line Breaks Inline Grammar
 */

inline.breaks = merge({}, inline.gfm, {
  br: replace(inline.br)('{2,}', '*')(),
  text: replace(inline.gfm.text)('{2,}', '*')()
});

/**
 * Inline Lexer & Compiler
 */

function InlineLexer(links, options) {
  this.options = options || marked.defaults;
  this.links = links;
  this.rules = inline.normal;
  this.renderer = this.options.renderer || new Renderer;
  this.renderer.options = this.options;

  if (!this.links) {
    throw new
      Error('Tokens array requires a `links` property.');
  }

  if (this.options.gfm) {
    if (this.options.breaks) {
      this.rules = inline.breaks;
    } else {
      this.rules = inline.gfm;
    }
  } else if (this.options.pedantic) {
    this.rules = inline.pedantic;
  }
}

/**
 * Expose Inline Rules
 */

InlineLexer.rules = inline;

/**
 * Static Lexing/Compiling Method
 */

InlineLexer.output = function(src, links, options) {
  var inline = new InlineLexer(links, options);
  return inline.output(src);
};

/**
 * Lexing/Compiling
 */

InlineLexer.prototype.output = function(src) {
  var out = ''
    , link
    , text
    , href
    , cap;

  while (src) {
    // escape
    if (cap = this.rules.escape.exec(src)) {
      src = src.substring(cap[0].length);
      out += cap[1];
      continue;
    }

    // autolink
    if (cap = this.rules.autolink.exec(src)) {
      src = src.substring(cap[0].length);
      if (cap[2] === '@') {
        text = cap[1].charAt(6) === ':'
          ? this.mangle(cap[1].substring(7))
          : this.mangle(cap[1]);
        href = this.mangle('mailto:') + text;
      } else {
        text = escape(cap[1]);
        href = text;
      }
      out += this.renderer.link(href, null, text);
      continue;
    }

    // inl_tooltip
    if (cap = this.rules.inl_tooltip.exec(src)) {
      src = src.substring(cap[0].length);
      title = this.output(cap[1]);
      text = Parser.parse(Lexer.lex(cap[2]));
      out += this.renderer.tooltip('d', title, text);
      continue;
    }

    // url (gfm)
    if (!this.inLink && (cap = this.rules.url.exec(src))) {
      src = src.substring(cap[0].length);
      text = escape(cap[1]);
      href = text;
      out += this.renderer.link(href, null, text);
      continue;
    }

    // tag
    if (cap = this.rules.tag.exec(src)) {
      if (!this.inLink && /^<a /i.test(cap[0])) {
        this.inLink = true;
      } else if (this.inLink && /^<\/a>/i.test(cap[0])) {
        this.inLink = false;
      }
      src = src.substring(cap[0].length);
      out += this.options.sanitize
        ? escape(cap[0])
        : cap[0];
      continue;
    }

    // link
    if (cap = this.rules.link.exec(src)) {
      src = src.substring(cap[0].length);
      this.inLink = true;
      out += this.outputLink(cap, {
        href: cap[2],
        title: cap[3]
      });
      this.inLink = false;
      continue;
    }

    // reflink, nolink
    if ((cap = this.rules.reflink.exec(src))
        || (cap = this.rules.nolink.exec(src))) {
      src = src.substring(cap[0].length);
      link = (cap[2] || cap[1]).replace(/\s+/g, ' ');
      link = this.links[link.toLowerCase()];
      if (!link || !link.href) {
        out += cap[0].charAt(0);
        src = cap[0].substring(1) + src;
        continue;
      }
      this.inLink = true;
      out += this.outputLink(cap, link);
      this.inLink = false;
      continue;
    }

    // strong
    if (cap = this.rules.strong.exec(src)) {
      src = src.substring(cap[0].length);
      out += this.renderer.strong(this.output(cap[2] || cap[1]));
      continue;
    }

    // em
    if (cap = this.rules.em.exec(src)) {
      src = src.substring(cap[0].length);
      out += this.renderer.em(this.output(cap[2] || cap[1]));
      continue;
    }

    // code
    if (cap = this.rules.code.exec(src)) {
      src = src.substring(cap[0].length);
      out += this.renderer.codespan(escape(cap[2], true));
      continue;
    }

    // br
    if (cap = this.rules.br.exec(src)) {
      src = src.substring(cap[0].length);
      out += this.renderer.br();
      continue;
    }

    // del (gfm)
    if (cap = this.rules.del.exec(src)) {
      src = src.substring(cap[0].length);
      out += this.renderer.del(this.output(cap[1]));
      continue;
    }

    // text
    if (cap = this.rules.text.exec(src)) {
      src = src.substring(cap[0].length);
      out += escape(this.smartypants(cap[0]));
      continue;
    }

    if (src) {
      throw new
        Error('Infinite loop on byte: ' + src.charCodeAt(0));
    }
  }

  return out;
};

/**
 * Compile Link
 */

InlineLexer.prototype.outputLink = function(cap, link) {
  var href = escape(link.href)
    , title = link.title ? escape(link.title) : null;

  return cap[0].charAt(0) !== '!'
    ? this.renderer.link(href, title, this.output(cap[1]))
    : this.renderer.image(href, title, escape(cap[1]));
};

/**
 * Smartypants Transformations
 */

InlineLexer.prototype.smartypants = function(text) {
  if (!this.options.smartypants) return text;
  return text
    // em-dashes
    .replace(/--/g, '\u2014')
    // opening singles
    .replace(/(^|[-\u2014/(\[{"\s])'/g, '$1\u2018')
    // closing singles & apostrophes
    .replace(/'/g, '\u2019')
    // opening doubles
    .replace(/(^|[-\u2014/(\[{\u2018\s])"/g, '$1\u201c')
    // closing doubles
    .replace(/"/g, '\u201d')
    // ellipses
    .replace(/\.{3}/g, '\u2026');
};

/**
 * Mangle Links
 */

InlineLexer.prototype.mangle = function(text) {
  var out = ''
    , l = text.length
    , i = 0
    , ch;

  for (; i < l; i++) {
    ch = text.charCodeAt(i);
    if (Math.random() > 0.5) {
      ch = 'x' + ch.toString(16);
    }
    out += '&#' + ch + ';';
  }

  return out;
};

/**
 * Renderer
 */

function Renderer(options) {
  this.options = options || {};
}

Renderer.prototype.code = function(code, lang, escaped, caption) {
  if (this.options.highlight) {
    var out = this.options.highlight(code, lang);
    if (out != null && out !== code) {
      escaped = true;
      code = out;
    }
  }

  // Increase global counter for SelectText
  codeBlockIdx++;

  return '<div style="display:inline-block"><div style="display:block">'
    + '<button type="button" class="btn btn-xs btn-warning float-right select-text" style="margin: 0; position: relative;"onclick="SelectText(\'codeBlock_'+codeBlockIdx+'\')">Select text</button></div>'
    + '<div class="pre-container"><pre><code id="codeBlock_'+codeBlockIdx+'" class="'
    + (lang?(''+this.options.langPrefix + escape(lang, true)):'')
    + '">'
    + (escaped ? code : escape(code, true))
    + '\n</code></pre>'
    + (caption ? '<span class="code-title">' + caption + '</span>': '')
    + '</div></div>\n';
};

Renderer.prototype.callout = function(level, symbol, floating, title, text) {
  level = level
        .replace(/\[\[r/g, 'danger')
        .replace(/\[\[b/g, 'info')
        .replace(/\[\[g/g, 'success')
        .replace(/\[\[y/g, 'warning')
        .replace(/\[\[d/g, 'default')

  if (symbol)
  {
    if (symbol == '!')
    {
      if (level == 'danger')
        glyph = '-exclamation-sign';
      else if (level == 'info')
        glyph = '-info-sign';
      else if (level == 'success')
        glyph = '-ok-sign';
      else if (level == 'warning')
        glyph = '-exclamation-sign';
      else if (level == 'default')
        glyph = '-hand-right';
    }
    else
    {
      glyph = symbol; // Use text directly.
    }

    symbol = '<span class="glyphicon glyphicon'+glyph+' gi-2x" style="vertical-align: middle; margin-right: 0.2em;"></span>'
  }
  else
    symbol = '';

  return '<div class="bs-callout bs-callout-'+level+' '+(floating ? 'float-right' : '') +'">'
    + '<h4>'
    + symbol
    + '<span style="vertical-align: middle;">' + striphtml(title) + '</span>'
    +'</h4>\n'
    + text
    + '</div>\n'
};

Renderer.prototype.tooltip = function(color, title, text) {

return '<span class="hinted" data-placement="auto top" data-toggle="tooltip"'
  + ' data-html="true" data-original-title="'+text+'">'+title+'</span>';
};

Renderer.prototype.expandable = function(level, title, text) {
  level = level
        .replace(/\[\[\+r/g, 'panel-primary')
        .replace(/\[\[\+b/g, 'panel-info')
        .replace(/\[\[\+g/g, 'panel-success')
        .replace(/\[\[\+y/g, 'panel-warning')
        .replace(/\[\[\+d/g, 'panel-default')

  var expandId = 'expandable_' + striphtml(title).toLowerCase().replace(/[^\w]+/g, '-');

  return '<div class="panel-group">'
  + '\n  <div class="panel '+level+'">'
  + '\n    <div class="panel-heading">'
  + '\n      <h4 class="panel-title">'
  + '\n        <a class="accordion-toggle collapsed" data-toggle="collapse" href="#'+ expandId +'">'+ title +'</a>'
  + '\n      </h4>'
  + '\n    </div>'
  + '\n    <div id="'+ expandId +'" class="panel-collapse collapse">'
  + '\n      <div class="panel-body">'+ text +'</div>'
  + '\n    </div>'
  + '\n  </div>'
  + '\n</div>'
};


Renderer.prototype.alert = function(level, floating, text) {
  level = level
        .replace(/\{\{r/g, 'alert-danger')
        .replace(/\{\{b/g, 'alert-info')
        .replace(/\{\{g/g, 'alert-success')
        .replace(/\{\{y/g, 'alert-warning')

  return '<div class="alert '+level+' '+(floating ? 'float-right' : '') +'">'
    + text
    + '</div>\n'
};

// Globals. Gh.
var questionNumber = 0;
var questionGroupNumber = 0;

Renderer.prototype.quiz = function(body) {
  var ret = '<div class="quiz-toolbar"><div>\n';
  questionGroupNumber++;

  while(match = /([XxVv])(>?) ([\S ]+)/.exec(body))
  {
    body = body.substring(match[0].length+1);

    parts = match[3].split('-->');
    answer = parts[0];
    tooltip = parts[1];

    correct = (match[1]=='v' || match[1]=='V')?'right':'wrong';
    var qId = questionGroupNumber+'.'+questionNumber;
    ret += '<input type="radio" id="q' + qId + '"';
    ret += ' class="answer-'+correct+'"';
    ret += ' name="quiz-'+questionGroupNumber+'">';
    ret += '<label for="q' + qId + '" class="quiz-label '+(match[2]?'quiz-float':'')+'"'
    if (tooltip)
    {
      tooltip = Parser.parse(Lexer.lex(tooltip));
      ret += ' data-toggle="tooltip" data-html="true" data-container="body" data-placement="auto top" data-trigger="click" title="'+tooltip+'"';
    }
    ret += '>'+answer+'</label>\n';
    questionNumber++;
  }
  ret += '</div></div>\n'

  return ret;
};

Renderer.prototype.quiz_multi = function(body) {
  var ret = '<div class="quiz-toolbar"><div>\n';
  questionGroupNumber++;

  while(match = /([XxVv])(>?) ([\S ]+)/.exec(body))
  {
    body = body.substring(match[0].length + 1);

    parts = match[3].split('-->');
    answer = parts[0];
    tooltip = parts[1];

    correct = (match[1]=='v' || match[1]=='V')?'right':'wrong';
    var qId = questionGroupNumber+'.'+questionNumber;
    ret += '<input type="checkbox" id="q' + qId + '"';
    ret += ' class="answer-'+correct+'"';
    ret += ' name="quiz-'+questionGroupNumber+'" style="display:none;">';
    ret += '<label for="q' + qId + '" class="quiz-label '+(match[2]?'quiz-float':'')+'"'
    if (tooltip)
    {
      tooltip = Parser.parse(Lexer.lex(tooltip));
      ret += ' data-toggle="tooltip" data-html="true" data-container="body" data-placement="auto top" data-trigger="click" title="'+tooltip+'"';
    }
    ret += '>'+answer+'</label>\n';
    questionNumber++;
  }
  ret += '</div></div>\n'

  return ret;
};

Renderer.prototype.blockquote = function(quote) {
  return '<blockquote>\n' + quote + '</blockquote>\n';
};

Renderer.prototype.html = function(html) {
  return html;
};

Renderer.prototype.heading = function(text, level, raw) {
  if (lastChapterDepth == 0)
  {
  	chapters += '<ul class="nav nav-stacked fixed" id="sidebar">'
  }

  if (lastChapterDepth && level > lastChapterDepth)
  {
  	chapters += Array(level - lastChapterDepth + 1).join('<ul class="nav nav-stacked">')
  }
  else if (lastChapterDepth && level < lastChapterDepth)
  {
	chapters += Array(lastChapterDepth-level + 1).join('</ul></li>')
  }
  lastChapterDepth = level;

  chapters += '<li><a href="#'
           + this.options.headerPrefix
           + raw.toLowerCase().replace(/[^\w]+/g, '-')
           + '">'+striphtml(text)+'</a>'

  return '<h'
    + level
    + ' id="'
    + this.options.headerPrefix
    + raw.toLowerCase().replace(/[^\w]+/g, '-')
    + '">'
    + text
    + '</h'
    + level
    + '>\n';
};

Renderer.prototype.hr = function() {
  return this.options.xhtml ? '<hr/>\n' : '<hr>\n';
};

Renderer.prototype.list = function(body, ordered) {
  var type = ordered ? 'ol' : 'ul';
  return '<' + type + '>\n' + body + '</' + type + '>\n';
};

Renderer.prototype.listitem = function(text) {
  return '<li>' + text + '</li>\n';
};

Renderer.prototype.paragraph = function(text) {
  return '<p>' + text + '</p>\n';
};

Renderer.prototype.table = function(header, body) {
  return '<table>\n'
    + '<thead>\n'
    + header
    + '</thead>\n'
    + '<tbody>\n'
    + body
    + '</tbody>\n'
    + '</table>\n';
};

Renderer.prototype.tablerow = function(content) {
  return '<tr>\n' + content + '</tr>\n';
};

Renderer.prototype.tablecell = function(content, flags) {
  var type = flags.header ? 'th' : 'td';
  var tag = flags.align
    ? '<' + type + ' style="text-align:' + flags.align + '">'
    : '<' + type + '>';
  return tag + content + '</' + type + '>\n';
};

// span level renderer
Renderer.prototype.strong = function(text) {
  return '<strong>' + text + '</strong>';
};

Renderer.prototype.em = function(text) {
  return '<em>' + text + '</em>';
};

Renderer.prototype.codespan = function(text) {
  return '<code>' + text + '</code>';
};

Renderer.prototype.br = function() {
  return this.options.xhtml ? '<br/>' : '<br>';
};

Renderer.prototype.del = function(text) {
  return '<del>' + text + '</del>';
};

Renderer.prototype.link = function(href, title, text) {
  if (this.options.sanitize) {
    try {
      var prot = decodeURIComponent(unescape(href))
        .replace(/[^\w:]/g, '')
        .toLowerCase();
    } catch (e) {
      return '';
    }
    if (prot.indexOf('javascript:') === 0 || prot.indexOf('vbscript:') === 0) {
      return '';
    }
  }
  var out = '<a href="' + href + '"';
  if (title) {
    out += ' title="' + title + '"';
  }
  out += '>' + text + '</a>';
  return out;
};

Renderer.prototype.image = function(href, title, text) {
  var out = '<img src="' + href + '" alt="' + text + '" class="img-responsive"';
  if (title) {
    out += ' title="' + title + '"';
  }
  out += this.options.xhtml ? '/>' : '>';
  out += '';
  return out;
};

/**
 * Parsing & Compiling
 */

function Parser(options) {
  this.tokens = [];
  this.token = null;
  this.options = options || marked.defaults;
  this.options.renderer = this.options.renderer || new Renderer;
  this.renderer = this.options.renderer;
  this.renderer.options = this.options;
}

/**
 * Static Parse Method
 */

Parser.parse = function(src, options, renderer) {
  var parser = new Parser(options, renderer);
  return parser.parse(src);
};

/**
 * Parse Loop
 */

Parser.prototype.parse = function(src) {
  this.inline = new InlineLexer(src.links, this.options, this.renderer);
  this.tokens = src.reverse();

  var out = '';
  while (this.next()) {
    out += this.tok();
  }

  return out;
};

/**
 * Next Token
 */

Parser.prototype.next = function() {
  return this.token = this.tokens.pop();
};

/**
 * Preview Next Token
 */

Parser.prototype.peek = function() {
  return this.tokens[this.tokens.length - 1] || 0;
};

/**
 * Parse Text Tokens
 */

Parser.prototype.parseText = function() {
  var body = this.token.text;

  while (this.peek().type === 'text') {
    body += '\n' + this.next().text;
  }

  return this.inline.output(body);
};

/**
 * Parse Current Token
 */

Parser.prototype.tok = function() {
  switch (this.token.type) {
    case 'space': {
      return '';
    }
    case 'hr': {
      return this.renderer.hr();
    }
    case 'heading': {
      return this.renderer.heading(
        this.inline.output(this.token.text),
        this.token.depth,
        this.token.text);
    }
    case 'code': {
      return this.renderer.code(this.token.text,
        this.token.lang,
        this.token.escaped,
        this.token.caption);
    }
    case 'code_start': {
      var code_text = this.token.text;
      var code_lang = this.token.lang;
      var code_escaped = this.token.escaped;
      var code_caption = '';
      while (this.next().type !== 'code_end') {
        	code_caption += this.tok();
      }
      return this.renderer.code(code_text,
        code_lang,
        code_escaped,
        code_caption);
    }
    case 'callout_start': {
    	var level = '';
      var symbol = '';
    	var floating = '';
    	var title = '';
    	var text = '';

    	level = this.token.level;
    	floating = this.token.floating;
      symbol = this.token.symbol;

      	while (this.next().type !== 'callout_title_end') {
        	title += this.tok();
      	}

      	while (this.next().type !== 'callout_end') {
        	text += this.tok();
      	}

    	return this.renderer.callout(level, symbol, floating,
    		title,
    		text);
    }

    case 'tooltip_start': {
      var color = '';
      var title = '';
      var text = '';

      color = this.token.color;
      while (this.next().type !== 'tooltip_title_end') {
        title += this.tok();
      }

      while (this.next().type !== 'tooltip_end') {
        text += this.tok();
      }

      return this.renderer.tooltip(color, title, text);
    }

    case 'expandable_start': {
      var level = '';
      var title = '';
      var text = '';

      level = this.token.level;

        while (this.next().type !== 'expandable_title_end') {
          title += this.tok();
        }

        while (this.next().type !== 'expandable_end') {
          text += this.tok();
        }

      return this.renderer.expandable(level,
        title,
        text);
    }

    case 'alert_start': {
      var level = '';
      var text = '';
      var floating = '';
      floating = this.token.floating;

      level = this.token.level;

        while (this.next().type !== 'alert_end') {
          text += this.tok();
        }

      return this.renderer.alert(level, floating,
        text);
    }
    case 'quiz': {
      return this.renderer.quiz(this.token.body);
    }
    case 'quiz_multi': {
      return this.renderer.quiz_multi(this.token.body);
    }
    case 'table': {
      var header = ''
        , body = ''
        , i
        , row
        , cell
        , flags
        , j;

      // header
      cell = '';
      for (i = 0; i < this.token.header.length; i++) {
        flags = { header: true, align: this.token.align[i] };
        cell += this.renderer.tablecell(
          this.inline.output(this.token.header[i]),
          { header: true, align: this.token.align[i] }
        );
      }
      header += this.renderer.tablerow(cell);

      for (i = 0; i < this.token.cells.length; i++) {
        row = this.token.cells[i];

        cell = '';
        for (j = 0; j < row.length; j++) {
          cell += this.renderer.tablecell(
            this.inline.output(row[j]),
            { header: false, align: this.token.align[j] }
          );
        }

        body += this.renderer.tablerow(cell);
      }
      return this.renderer.table(header, body);
    }
    case 'blockquote_start': {
      var body = '';

      while (this.next().type !== 'blockquote_end') {
        body += this.tok();
      }

      return this.renderer.blockquote(body);
    }
    case 'list_start': {
      var body = ''
        , ordered = this.token.ordered;

      while (this.next().type !== 'list_end') {
        body += this.tok();
      }

      return this.renderer.list(body, ordered);
    }
    case 'list_item_start': {
      var body = '';

      while (this.next().type !== 'list_item_end') {
        body += this.token.type === 'text'
          ? this.parseText()
          : this.tok();
      }

      return this.renderer.listitem(body);
    }
    case 'loose_item_start': {
      var body = '';

      while (this.next().type !== 'list_item_end') {
        body += this.tok();
      }

      return this.renderer.listitem(body);
    }
    case 'html': {
      var html = !this.token.pre && !this.options.pedantic
        ? this.inline.output(this.token.text)
        : this.token.text;
      return this.renderer.html(html);
    }
    case 'paragraph': {
      return this.renderer.paragraph(this.inline.output(this.token.text));
    }
    case 'text': {
      return this.renderer.paragraph(this.parseText());
    }
  }
};

/**
 * Helpers
 */

function escape(html, encode) {
  return html
    .replace(!encode ? /&(?!#?\w+;)/g : /&/g, '&amp;')
    .replace(/</g, '&lt;')
    .replace(/>/g, '&gt;')
    .replace(/"/g, '&quot;')
    .replace(/'/g, '&#39;');
}

function unescape(html) {
  return html.replace(/&([#\w]+);/g, function(_, n) {
    n = n.toLowerCase();
    if (n === 'colon') return ':';
    if (n.charAt(0) === '#') {
      return n.charAt(1) === 'x'
        ? String.fromCharCode(parseInt(n.substring(2), 16))
        : String.fromCharCode(+n.substring(1));
    }
    return '';
  });
}

function replace(regex, opt) {
  regex = regex.source;
  opt = opt || '';
  return function self(name, val) {
    if (!name) return new RegExp(regex, opt);
    val = val.source || val;
    val = val.replace(/(^|[^\[])\^/g, '$1');
    regex = regex.replace(name, val);
    return self;
  };
}

function noop() {}
noop.exec = noop;

function merge(obj) {
  var i = 1
    , target
    , key;

  for (; i < arguments.length; i++) {
    target = arguments[i];
    for (key in target) {
      if (Object.prototype.hasOwnProperty.call(target, key)) {
        obj[key] = target[key];
      }
    }
  }

  return obj;
}


/**
 * Marked
 */

function marked(src, opt, callback) {
  if (callback || typeof opt === 'function') {
    if (!callback) {
      callback = opt;
      opt = null;
    }

    opt = merge({}, marked.defaults, opt || {});

    var highlight = opt.highlight
      , tokens
      , pending
      , i = 0;

    try {
      tokens = Lexer.lex(src, opt)
    } catch (e) {
      return callback(e);
    }

    pending = tokens.length;

    var done = function(err) {
      if (err) {
        opt.highlight = highlight;
        return callback(err);
      }

      var out;

      try {
        out = Parser.parse(tokens, opt);
      } catch (e) {
        err = e;
      }

      opt.highlight = highlight;

      return err
        ? callback(err)
        : callback(null, out);
    };

    if (!highlight || highlight.length < 3) {
      return done();
    }

    delete opt.highlight;

    if (!pending) return done();

    for (; i < tokens.length; i++) {
      (function(token) {
        if (token.type !== 'code') {
          return --pending || done();
        }
        return highlight(token.text, token.lang, function(err, code) {
          if (err) return done(err);
          if (code == null || code === token.text) {
            return --pending || done();
          }
          token.text = code;
          token.escaped = true;
          --pending || done();
        });
      })(tokens[i]);
    }

    return;
  }
  try {
    if (opt) opt = merge({}, marked.defaults, opt);
    return Parser.parse(Lexer.lex(src, opt), opt);
  } catch (e) {
    e.message += '\nPlease report this to https://github.com/chjj/marked.';
    if ((opt || marked.defaults).silent) {
      return '<p>An error occured:</p><pre>'
        + escape(e.message + '', true)
        + '</pre>';
    }
    throw e;
  }
}

/**
 * Options
 */

marked.options =
marked.setOptions = function(opt) {
  merge(marked.defaults, opt);
  return marked;
};

marked.defaults = {
  gfm: true,
  tables: true,
  breaks: false,
  pedantic: false,
  sanitize: false,
  smartLists: false,
  silent: false,
  highlight: null,
  langPrefix: 'lang-',
  smartypants: false,
  headerPrefix: '',
  renderer: new Renderer,
  xhtml: false
};

/**
 * Expose
 */

marked.Parser = Parser;
marked.parser = Parser.parse;

marked.Renderer = Renderer;

marked.Lexer = Lexer;
marked.lexer = Lexer.lex;

marked.InlineLexer = InlineLexer;
marked.inlineLexer = InlineLexer.output;

marked.parse = marked;

if (typeof module !== 'undefined' && typeof exports === 'object') {
  module.exports = marked;
} else if (typeof define === 'function' && define.amd) {
  define(function() { return marked; });
} else {
  this.marked = marked;
}

}).call(function() {
  return this || (typeof window !== 'undefined' ? window : global);
}());


function SelectText(element) {
    var doc = document
        , text = doc.getElementById(element)
        , range, selection
    ;
    if (doc.body.createTextRange) {
        range = document.body.createTextRange();
        range.moveToElementText(text);
        range.select();
    } else if (window.getSelection) {
        selection = window.getSelection();
        range = document.createRange();
        range.selectNodeContents(text);
        selection.removeAllRanges();
        selection.addRange(range);
    }
}


;(function(window, document) {

  // Only fiddle with DOM if it's not pre-rendered.
  if (document.isPreRendered === undefined || document.isPreRendered == false)
  {
    // Hide body until we're done fiddling with the DOM
    document.body.style.display = 'none';

    //////////////////////////////////////////////////////////////////////
    //
    // Shims for IE < 9
    //

    document.head = document.getElementsByTagName('head')[0];

    if (!('getElementsByClassName' in document)) {
      document.getElementsByClassName = function(name) {
        function getElementsByClassName(node, classname) {
          var a = [];
          var re = new RegExp('(^| )'+classname+'( |$)');
          var els = node.getElementsByTagName("*");
          for(var i=0,j=els.length; i<j; i++)
              if(re.test(els[i].className))a.push(els[i]);
          return a;
        }
        return getElementsByClassName(document.body, name);
      }
    }

    //////////////////////////////////////////////////////////////////////
    //
    // Get user elements we need
    //

    var markdownEl = document.getElementsByTagName('xmp')[0] || document.getElementsByTagName('textarea')[0],
        titleEl = document.getElementsByTagName('title')[0],
        scriptEls = document.getElementsByTagName('script'),
        navbarEl = document.getElementsByClassName('navbar')[0];



  /*
  var scriptEl = document.createElement('script');
  scriptEl.src = '../strapdown/vendor/jquery-1.11.2.min.js';
  scriptEl.type = "application/javascript"
  document.body.appendChild(scriptEl);

  var scriptEl = document.createElement('script');
  scriptEl.src = '../strapdown/vendor/bootstrap.min.js';
  scriptEl.type = "application/javascript"
  document.body.appendChild(scriptEl);

  var scriptEl = document.createElement('script');
  scriptEl.src = '../highlight/highlight.pack.js';
  scriptEl.type = "application/javascript"
  document.body.appendChild(scriptEl);

  <!--script src="https://ajax.googleapis.com/ajax/libs/jquery/1.11.2/jquery.min.js"></script>
  <script src="../strapdown/vendor/bootstrap.min.js"></script>
  <script src="../highlight/highlight.pack.js"></script>
  <link rel="stylesheet" href="../highlight/styles/zenburn.css"-->
  */
    //////////////////////////////////////////////////////////////////////
    //
    // <body> stuff
    //

    var markdown = markdownEl.textContent || markdownEl.innerText;


    // Insert stuff to align sidebar and content
    var pageContainer = document.createElement('div');
    pageContainer.className = 'container';

    document.body.replaceChild(pageContainer, markdownEl);

    // Insert navbar if there's none
    var newNode = document.createElement('nav');
    newNode.className = 'navbar navbar-default navbar-static-top';
    if (!navbarEl && titleEl) {
      newNode.innerHTML = '<div class="container-fluid"> <div class="navbar-header">  <div id="headline" class="navbar-brand"> </div> </div> </div>';
      //document.body.insertBefore(newNode, document.body.firstChild);
      pageContainer.appendChild(newNode);
      var title = titleEl.innerHTML;
      var headlineEl = document.getElementById('headline');
      if (headlineEl)
        headlineEl.innerHTML = title;
    }


    var newNode = document.createElement('div');
    newNode.className = 'container';

    var rowNode = document.createElement('div');
    rowNode.className = 'row row-offcanvas row-offcanvas-left';
    newNode.appendChild(rowNode);
    var contentNode = document.createElement('div');
    contentNode.className = 'col-xs-12 col-sm-9';
    contentNode.id = 'content';
    rowNode.appendChild(contentNode);
    pageContainer.appendChild(newNode);


    // Sidebar
    var sideBar = document.createElement('div');
    sideBar.className = 'col-xs-4 col-sm-2 sidebar-offcanvas bs-docs-sidebar hidden-print';
    sideBar.id = 'sidebar-overview';
    rowNode.appendChild(sideBar);



    //////////////////////////////////////////////////////////////////////
    //
    // Markdown!
    //

    // Generate Markdown
    var html = marked(markdown);
    document.getElementById('content').innerHTML = html;

    // generate table of contents.
    chapters += Array(lastChapterDepth+1).join('</ul>');
    document.getElementById('sidebar-overview').innerHTML = chapters;
  /*
    // Prettify
    var codeEls = document.getElementsByTagName('code');
    for (var i=0, ii=codeEls.length; i<ii; i++) {
      var codeEl = codeEls[i];
      var lang = codeEl.className;
      codeEl.className = 'prettyprint lang-' + lang;
    }
    prettyPrint();
  */

    // Style tables
    var tableEls = document.getElementsByTagName('table');
    for (var i=0, ii=tableEls.length; i<ii; i++) {
      var tableEl = tableEls[i];
      tableEl.className = 'table table-striped table-bordered';
    }

    // All done - show body
    document.body.style.display = '';

    /*
     * Code highlighting
     */
    hljs.configure({
      tabReplace: '  '
    })
    hljs.initHighlightingOnLoad();

    // Add modal template for zoomed-in images.
    var modalDivHtml = '\
      <div class="modal" id="imagemodal" tabindex="-1" role="dialog" aria-labelledby="myModalLabel" aria-hidden="true">\
        <div class="vertical-alignment-helper">\
          <div class="modal-dialog vertical-align-center">\
             <div class="modal-dialog">\
               <div class="modal-content" style="margin-left: auto;margin-right: auto;">\
                 <div class="modal-body">\
                   <button type="button" class="close" data-dismiss="modal"><span aria-hidden="true">&times;</span><span class="sr-only">Close</span></button>\
                   <img src="" class="imagepreview img-responsive" >\
                 </div>\
              </div>\
            </div>\
          </div>\
        </div>\
      </div>';
    $('body').append(modalDivHtml);
  }

  /*
   * Utility / support functions and event bindings.
   */
   // Activate/deactivate items in the table of contents based on scroll position
  $('body').scrollspy({
      target: '.bs-docs-sidebar',
      offset: 40
  });

  // Bind tooltip function to tooltip elements.
  $(document).ready(function(){
    $('[data-toggle="tooltip"]').tooltip();
  });

  $('body').on('click', function (e) {
      $('[data-toggle="tooltip"]').each(function () {
          //the 'is' for buttons that trigger popups
          //the 'has' for icons within a button that triggers a popup
          if (!$(this).is(e.target))
            if (this.htmlFor != e.target.id) // In case the radio
              if ($(this).has(e.target).length === 0)
                if ($('.tooltip').has(e.target).length === 0)
                {
                  $(this).tooltip('hide');
                }
      });
  });



  $(function() {
    $('.modal-pop').on('click', function() {
      $('.imagepreview').attr('src', $(this).find('img').attr('src'));

      $('#imagemodal').modal('show');
    });

    // Allow max sized modal
    $('#imagemodal').on('shown.bs.modal', function () {
      imgWidth = $(this).find('img').prop("naturalWidth") + 60;
      //$('<img/>').attr('src', $(this).find('img').attr('src')).load(function(){imgWidth=this.width+40});
      modalWidth = $('#imagemodal').width() - 60;
      minWidth = Math.min(imgWidth, modalWidth);
      //$(this).find('.modal-dialog').css('max-width', minWidth+'px');

      $(this).find('.modal-dialog').css({width: minWidth+'px',
                                 height:'auto',
                                'max-height':'100%'});
    });
  });

  /*
   * Override link clicks on sidebar to allow jumping into collapsed
   * boxes. Expands them first.
   */

  /* Determine if an element has a parent that is collapsible */
  function findUpCollapse(el) {
      while (el.parent()[0])
      {
        el = el.parent();
        if (el.hasClass('panel-collapse'))
            return el;
      }
      return null;
  }

  /* Override <a> clicks on sidebar with magic */
  $("#sidebar-overview").find('a').bind("click", function(e) {
    var href;
    var target = e.target || e.srcElement;

    e.preventDefault(); // Don't try to jump, please
    href = target.getAttribute('href');
    collapse = findUpCollapse($(href));

    /* Animate browser movement to an element with href #id */
    function scrollToLink(href) {
        $('html, body').animate({
            scrollTop: $(href).offset().top
        }, 200);
    }

    /* If collapsed, set event on uncollapse done and do uncollapse */
    if (collapse && !collapse.hasClass('in'))
    {
      collapse.on("shown.bs.collapse", function(){
        scrollToLink(href);
        collapse.off("shown.bs.collapse");
      });
      collapse.collapse('show');
    }
    else
    {
      scrollToLink(href);
    }
  });


  /* Prevent jittery sidebar (fixed pos) on IE with mousewheel
   * https://coderwall.com/p/hlqqia/ie-fix-for-jumpy-fixed-bacground
   */
  if(navigator.userAgent.match(/Trident\/7\./)) { // if IE
      $('body').on("mousewheel", function () {
          // remove default behavior
          event.preventDefault();

          //scroll without smoothing
          var wheelDelta = event.wheelDelta;
          var currentScrollPosition = window.pageYOffset;
          window.scrollTo(0, currentScrollPosition - wheelDelta);
      });
  }

})(window, document);
