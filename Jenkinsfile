// Jenkinsfile

pipeline {
    agent any

    environment {
        IMAGE_PATH = "${env.GIT_URL.replace('https://lab.ssafy.com/', '').replace('.git', '').toLowerCase()}"
        REGISTRY = "lab.ssafy.com"
        IMAGE_NAME = "${REGISTRY}/${IMAGE_PATH}:${env.BUILD_NUMBER}"
        // (중요) 아래 EC2_HOST 변수에 자신의 EC2 공개 주소를 입력해야 합니다.
        EC2_HOST = "52.78.193.190" 
    }

    stages {
        stage('Build') {
            steps {
                script {
                    checkout scm
                    withCredentials([usernamePassword(credentialsId: 'gitlab-registry-credentials', usernameVariable: 'REGISTRY_USER', passwordVariable: 'REGISTRY_PASSWORD')]) {
                        sh "echo ${REGISTRY_PASSWORD} | docker login ${REGISTRY} -u ${REGISTRY_USER} --password-stdin"
                    }
                    sh "docker build -t ${IMAGE_NAME} ."
                    sh "docker push ${IMAGE_NAME}"
                }
            }
        }

        stage('Deploy') {
            steps {
                script {
                    echo "Deploying to ${EC2_HOST} via SSH..."
                    // Jenkins에 등록한 EC2 접속용 SSH 키 인증 정보를 사용합니다.
                    sshagent(credentials: ['ssafy-ec2-key']) {
                        // ssh로 EC2에 접속해서 배포 스크립트를 원격으로 실행합니다.
                        sh """
                            ssh -o StrictHostKeyChecking=no ubuntu@${EC2_HOST} "
                                export IMAGE_TO_DEPLOY=${IMAGE_NAME} &&
                                cd ~/S13P21A507 &&
                                docker-compose down &&
                                docker-compose up --pull -d &&
                                docker image prune -af
                            "
                        """
                    }
                    echo "Deploy Complete."
                }
            }
        }
    }

    post {
        always {
            echo "Pipeline finished."
        }
    }
}