$( document ).ready(function() {



    $('.enabled').click(function(){
        var clickBtnValue = $(this).attr('class').split(' ')[0];
        var ajaxurl = 'filesave.php',
        data =  {'action': clickBtnValue};
        $.post(ajaxurl, data, function (response) {
            // Response div goes here.
            $('.' + clickBtnValue).siblings().removeClass('enabled');
            $('.' + clickBtnValue).siblings().addClass('disabled');
            $('.' + clickBtnValue).siblings().css('background','grey');
            $('.' + clickBtnValue).addClass('disabled')
            $('.' + clickBtnValue).css('border-color','black');
        });
    });


    $('.call-rob').click(function(){
        $('#wrapper').show();
    });

    $('.no').click(function(){
        $('#wrapper').hide();
    });

    $('.yes').click(function(){
        $('#wrapper').hide();
    });
    

});
