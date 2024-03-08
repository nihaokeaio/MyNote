#pragma once
#include <string>
#include <vector>
using namespace std;


struct ListNode {
	int val;
	ListNode* next;
	ListNode() : val(0), next(nullptr) {}
	ListNode(int x) : val(x), next(nullptr) {}
	ListNode(int x, ListNode* next) : val(x), next(next) {}
};

struct TreeNode
{
	int val;
	TreeNode* left;
	TreeNode* right;
	TreeNode():val(0),left(nullptr),right(nullptr){}
	TreeNode(int x) :val(x), left(nullptr), right(nullptr) {}
};

class MyAlgorithms
{
public:
	///�ַ���ƥ���㷨
	//\�ַ���ƥ���㷨:��txt��Ѱ��pat���������±꣬��û��ƥ���򷵻�-1
	//չʾ��ͨ�������㷨
	int serachStr(const string& txt, const string& pat);

	//\KMP״̬���㷨
	//\�ĵ�����:https://zhuanlan.zhihu.com/p/83334559
	int serachStrKMP(const string& txt, const string& pat);
	void createKMPVec(vector<vector<int>>& nums, const string& pat);




	///�������
	///��ת����
	ListNode* reverseList(ListNode* head);

	///ɾ������ĵ�����N���ڵ�
	ListNode* removeNthFromEnd(ListNode* head, int n);




	///������
	///�ݹ����
	
private:
};
