    $(window).ready(function() {
      $("a:contains('Load in Sensing Estimator')").addClass('btn btn-primary');
      $("a[href^='https://dev.ti.com/gallery']").attr("target", "_blank");
      $("a[href^='http://dev.ti.com/gallery']").attr("target", "_blank");
      $("a[href^='http://dev.ti.com/mmWaveSensingEstimator']").attr("target", "_blank");
      $("a[href^='http://dev.ti.com/mmWaveDemoVisualizer']").attr("target", "_blank");
      $("a[href^='https://dev.ti.com/mmWaveSensingEstimator']").attr("target", "_blank");
      $("a[href^='https://dev.ti.com/mmWaveDemoVisualizer']").attr("target", "_blank");
      var re = new RegExp("(?:.*?link=)(.*)");
      $("a[href*='dev.ti.com/tirex/#']").each(function() {
        var link = $(this);
        var href = link.attr("href");
        link.removeAttr("href");
        var item = re.exec(href)[1];
        item = item.replace(/%252F/g, "/");
        item = item.replace(/%2520/g, " ");
        item = item.replace(/%2F/g, "/");
        item = item.replace(/%20/g, " ");
        //var path = item.split("/");
        //var mergedPath = path[path.length - 2] + "/" + path[path.length - 1];
        var mergedPath = item;
        link.on("click", function() {
          load_link(mergedPath);
        });
      });

      $('body').scrollspy({
        target: '.bs-docs-sidebar',
        offset: 40
      });

      var modalImageWidth = 0;

      $('.img-responsive').on('click', function() {
        $('.imagepreview').attr('src', $(this).attr('src'));
        modalImageWidth = $(this).prop('naturalWidth');

        $('#imagemodal').modal('show');
      });

      // Allow max sized modal
      $('#imagemodal').on('shown.bs.modal', function () {
        console.log($(this).find('.imagepreview').get(0));
        imgWidth = modalImageWidth + 60;
        //$('<img/>').attr('src', $(this).find('img').attr('src')).load(function(){imgWidth=this.width+40});
        modalWidth = $('#imagemodal').width() - 60;
        minWidth = Math.min(imgWidth, modalWidth);
        //$(this).find('.modal-dialog').css('max-width', minWidth+'px');

        $(this).find('.modal-dialog').css({width: minWidth+'px',
                                   height:'auto',
                                  'max-height':'100%'});
      });
    });

