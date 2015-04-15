

<?php 

if (isset($_POST['action'])) {
  $content = $_POST['action'] . ',';
echo $_SERVER['DOCUMENT_ROOT'] . "/interface/feedback.txt";
  $fp = fopen($_SERVER['DOCUMENT_ROOT'] . "/interface/feedback.txt","a");
  fwrite($fp,$content);
  fclose($fp);
}


?>


